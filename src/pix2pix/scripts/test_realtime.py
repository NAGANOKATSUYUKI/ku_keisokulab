import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
from torchvision import transforms
import matplotlib.pyplot as plt
from PIL import Image as PILImage
from models import create_model
from options.test_options import TestOptions

try:
    import wandb
except ImportError:
    print('Warning: wandb package cannot be found. The option "--use_wandb" will result in error.')

bridge = CvBridge()
generated_image_pub = None  # パブリッシャをグローバル変数として宣言

def infra_listener():
    global generated_image_pub
    rospy.init_node('infra_listener', anonymous=True)
    
    rospy.Subscriber("/z/edge_image", ROSImage, infra_callback)  #from edges_realtime.py
    # rospy.Subscriber("/z/image2movie_infra", ROSImage, infra_callback)
    # rospy.Subscriber("/camera/infra/image_raw", ROSImage, infra_callback)
    
    generated_image_pub = rospy.Publisher('/z/generated_image', ROSImage, queue_size=10)
    rospy.loginfo("Subscribed to 'infra' topic and publishing to 'generated' topic")
    rospy.spin()

def infra_callback(data):
    rospy.loginfo("Received an infra image")
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        
        # 画像をモデルの入力形式に変換
        input_image = load_image(pil_image)
        # 画像をモデルに渡してテスト
        model.set_input({'A': input_image, 'B': input_image, 'A_paths': 'ROS_image', 'B_paths': 'ROS_image'})
        model.test()
        visuals = model.get_current_visuals()
        # 生成された画像を取得
        fake_image_tensor = visuals.get('fake_B')
        if fake_image_tensor is not None:
            fake_image = tensor_to_image(fake_image_tensor[0])
            fake_image = cv2.resize(fake_image, (640, 480), interpolation=cv2.INTER_NEAREST_EXACT)

            fake_image = cv2.cvtColor(fake_image, cv2.COLOR_RGB2BGR)
            cv2.imshow("Generated Image", fake_image)
            cv2.waitKey(1) 

            # 画像をROS Imageメッセージに変換
            ros_image = bridge.cv2_to_imgmsg(fake_image, encoding="rgb8")
            generated_image_pub.publish(ros_image)

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def tensor_to_image(tensor):
    image = tensor.cpu().detach().numpy()
    image = (image + 1) / 2 * 255
    image = image.astype(np.uint8).transpose(1, 2, 0)
    return image

def load_image(pil_image):
    transform = transforms.Compose([
        transforms.Resize((512, 512)),  # 必要に応じてサイズを調整
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])
    return transform(pil_image).unsqueeze(0)  # バッチ次元を追加

if __name__ == '__main__':
    opt = TestOptions().parse()  # テストオプションを取得
    # opt.dataroot = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/kuams2_data/data_0625'  # 必要な場合、適切なパスを設定
    opt.dataroot = '/home/keisoku/Robot_data/infra'
    opt.num_threads = 0
    opt.batch_size = 1
    opt.serial_batches = True
    opt.no_flip = True
    opt.display_id = -1

    model = create_model(opt)
    model.setup(opt)

    # ロガーの初期化
    if opt.use_wandb:
        wandb_run = wandb.init(project=opt.wandb_project_name, name=opt.name, config=opt) if not wandb.run else wandb.run
        wandb_run._label(repo='CycleGAN-and-pix2pix')

    try:
        infra_listener()
    except rospy.ROSInterruptException:
        pass
