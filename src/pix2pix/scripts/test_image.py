import os
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import torch
from torchvision import transforms
from options.test_options import TestOptions
from models import create_model

try:
    import wandb
except ImportError:
    print('Warning: wandb package cannot be found. The option "--use_wandb" will result in error.')

def tensor_to_image(tensor):
    image = tensor.cpu().detach().numpy()
    image = (image + 1) / 2 * 255
    image = image.astype(np.uint8).transpose(1, 2, 0)
    return image

def display_fake_image(fake_tensor):
    if fake_tensor is not None:
        image = tensor_to_image(fake_tensor)  # 画像を変換
        plt.imshow(image)
        plt.title('fake_B')
        plt.axis('off')
        plt.show()
    else:
        print("fake_B が見つかりません")

def load_image(image_path):
    image = Image.open(image_path).convert('RGB')
    transform = transforms.Compose([
        transforms.Resize((256, 256)),  # 必要に応じてサイズを調整
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])
    return transform(image).unsqueeze(0)  # バッチ次元を追加

if __name__ == '__main__':
    opt = TestOptions().parse()  # テストオプションを取得
    opt.dataroot = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/kuams2_data/data_0625'  # 必要な場合、適切なパスを設定
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

    # 画像をロード
    image_path = '/home/keisoku/pytorch-CycleGAN-and-pix2pix/datasets/kuams2_data/data_0625/B/train/001.png'  # ここに画像のパスを指定
    input_image = load_image(image_path)

    # モデルに入力を設定してテスト
    AtoB = opt.direction == 'AtoB'
    model.set_input({'A': input_image, 'B': input_image, 'A_paths': image_path, 'B_paths': image_path})  # 両方のキーを設定
    model.test()
    visuals = model.get_current_visuals()

    # 生成された画像を表示
    fake_image_tensor = visuals.get('fake_B')
    if fake_image_tensor is not None:
        display_fake_image(fake_image_tensor[0])
    else:
        print('生成された画像が見つかりません')
