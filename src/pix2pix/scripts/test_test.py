#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as transforms
import functools

# UnetGeneratorクラスの定義
class UnetGenerator(nn.Module):
    """Create a Unet-based generator"""

    def __init__(self, input_nc, output_nc, num_downs, ngf=64, norm_layer=nn.BatchNorm2d, use_dropout=False):
        """Construct a Unet generator
        Parameters:
            input_nc (int)  -- the number of channels in input images
            output_nc (int) -- the number of channels in output images
            num_downs (int) -- the number of downsamplings in UNet. For example, # if |num_downs| == 7,
                                image of size 128x128 will become of size 1x1 # at the bottleneck
            ngf (int)       -- the number of filters in the last conv layer
            norm_layer      -- normalization layer

        We construct the U-Net from the innermost layer to the outermost layer.
        It is a recursive process.
        """
        super(UnetGenerator, self).__init__()
        # construct unet structure
        unet_block = UnetSkipConnectionBlock(ngf * 8, ngf * 8, input_nc=None, submodule=None, norm_layer=norm_layer, innermost=True)  # add the innermost layer
        for i in range(num_downs - 5):          # add intermediate layers with ngf * 8 filters
            unet_block = UnetSkipConnectionBlock(ngf * 8, ngf * 8, input_nc=None, submodule=unet_block, norm_layer=norm_layer, use_dropout=use_dropout)
        # gradually reduce the number of filters from ngf * 8 to ngf
        unet_block = UnetSkipConnectionBlock(ngf * 4, ngf * 8, input_nc=None, submodule=unet_block, norm_layer=norm_layer)
        unet_block = UnetSkipConnectionBlock(ngf * 2, ngf * 4, input_nc=None, submodule=unet_block, norm_layer=norm_layer)
        unet_block = UnetSkipConnectionBlock(ngf, ngf * 2, input_nc=None, submodule=unet_block, norm_layer=norm_layer)
        self.model = UnetSkipConnectionBlock(output_nc, ngf, input_nc=input_nc, submodule=unet_block, outermost=True, norm_layer=norm_layer)  # add the outermost layer

    def forward(self, input):
        """Standard forward"""
        return self.model(input)

class UnetSkipConnectionBlock(nn.Module):
    """Defines the Unet submodule with skip connection.
        X -------------------identity----------------------
        |-- downsampling -- |submodule| -- upsampling --|
    """

    def __init__(self, outer_nc, inner_nc, input_nc=None,
                 submodule=None, outermost=False, innermost=False, norm_layer=nn.BatchNorm2d, use_dropout=False):
        """Construct a Unet submodule with skip connections.

        Parameters:
            outer_nc (int) -- the number of filters in the outer conv layer
            inner_nc (int) -- the number of filters in the inner conv layer
            input_nc (int) -- the number of channels in input images/features
            submodule (UnetSkipConnectionBlock) -- previously defined submodules
            outermost (bool)    -- if this module is the outermost module
            innermost (bool)    -- if this module is the innermost module
            norm_layer          -- normalization layer
            use_dropout (bool)  -- if use dropout layers.
        """
        super(UnetSkipConnectionBlock, self).__init__()
        self.outermost = outermost
        if type(norm_layer) == functools.partial:
            use_bias = norm_layer.func == nn.InstanceNorm2d
        else:
            use_bias = norm_layer == nn.InstanceNorm2d
        if input_nc is None:
            input_nc = outer_nc
        downconv = nn.Conv2d(input_nc, inner_nc, kernel_size=4,
                             stride=2, padding=1, bias=use_bias)
        downrelu = nn.LeakyReLU(0.2, True)
        downnorm = norm_layer(inner_nc)
        uprelu = nn.ReLU(True)
        upnorm = norm_layer(outer_nc)

        if outermost:
            upconv = nn.ConvTranspose2d(inner_nc * 2, outer_nc,
                                        kernel_size=4, stride=2,
                                        padding=1)
            down = [downconv]
            up = [uprelu, upconv, nn.Tanh()]
            model = down + [submodule] + up
        elif innermost:
            upconv = nn.ConvTranspose2d(inner_nc, outer_nc,
                                        kernel_size=4, stride=2,
                                        padding=1, bias=use_bias)
            down = [downrelu, downconv]
            up = [uprelu, upconv, upnorm]
            model = down + up
        else:
            upconv = nn.ConvTranspose2d(inner_nc * 2, outer_nc,
                                        kernel_size=4, stride=2,
                                        padding=1, bias=use_bias)
            down = [downrelu, downconv, downnorm]
            up = [uprelu, upconv, upnorm]

            if use_dropout:
                model = down + [submodule] + up + [nn.Dropout(0.5)]
            else:
                model = down + [submodule] + up

        self.model = nn.Sequential(*model)

    def forward(self, x):
        if self.outermost:
            return self.model(x)
        else:   # add skip connections
            return torch.cat([x, self.model(x)], 1)


class Pix2PixNode:
    def __init__(self):
        rospy.init_node('pix2pix_node', anonymous=True)
        self.bridge = CvBridge()

        # Realsenseカメラからの赤外線画像をサブスクライブ
        self.image_sub = rospy.Subscriber('/camera/infra/image_raw', Image, self.image_callback, queue_size=10)

        # Generatorモデルをロード
        self.pix2pix_model = self.load_model('/home/keisoku/pytorch-CycleGAN-and-pix2pix/checkpoints/Data_edge/latest_net_G.pth')

        # カラー化された画像をパブリッシュ
        self.image_pub = rospy.Publisher('/colorized_image', Image, queue_size=10)

    def load_model(self, model_path):
        model = UnetGenerator(input_nc=3, output_nc=3, num_downs=8, ngf=64, norm_layer=nn.BatchNorm2d, use_dropout=False)
        model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        model.eval()
        return model

    def image_callback(self, msg):
        try:
            # ROS Image messageをOpenCV画像に変換
            infrared_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # 画像をRGB形式に変換
            rgb_image = cv2.cvtColor(infrared_image, cv2.COLOR_GRAY2RGB)

            # 画像の前処理（リサイズなど）
            resized_image = cv2.resize(rgb_image, (256, 256))

            # 正規化
            input_data = resized_image.astype('float32') / 255.0

            # PyTorchのTensorに変換し、チャンネルを先頭に配置
            input_tensor = transforms.ToTensor()(input_data).unsqueeze(0)

            # Generatorを使って画像を生成
            colorized_tensor = self.generate_image(input_tensor)

            # TensorをNumPy配列に変換し、画素値をスケーリングして uint8 に変換
            colorized_image_scaled = (colorized_tensor.squeeze(0).permute(1, 2, 0).numpy() * 255).astype(np.uint8)

            # BGRからRGB変換
            colorized_image_scaled = cv2.cvtColor(colorized_image_scaled, cv2.COLOR_BGR2RGB)

            # カラー画像をROS Imageに変換
            try:
                colorized_image_msg = self.bridge.cv2_to_imgmsg(colorized_image_scaled, encoding='rgb8')
                self.image_pub.publish(colorized_image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"CVBridge Error: {e}")

            # カラー画像をリアルタイムで表示
            cv2.imshow('Colorized Image', colorized_image_scaled)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"Exception in image_callback: {e}")

    def generate_image(self, input_tensor):
        # Generatorモデルでカラー画像に変換
        with torch.no_grad():
            generated_tensor = self.pix2pix_model(input_tensor)

        return generated_tensor

def main():
    pix2pix_node = Pix2PixNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
