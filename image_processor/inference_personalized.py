from mmseg.apis import init_model, inference_model, show_result_pyplot

config_path = '/home/vinicius/Desktop/ic_files/mmsegCodes/mmsegmentation/configs/segformer/segformer_mit-b2_8xb1-160k_cityscapes-1024x1024.py'
checkpoint_path = '/home/vinicius/ros2_ws/src/image_processor/Models&Configs/iter_160000WPT.pth'
img_path = '/home/vinicius/ros2_ws/src/image_processor/images_source/000000001.png'


model = init_model(config_path, checkpoint_path, device='cuda:0')
result = inference_model(model, img_path)
vis_iamge = show_result_pyplot(model, img_path, result, show=False, with_labels=False, draw_pred=True, save_dir="/home/vinicius/ros2_ws/src/image_processor/masks_images/pred", out_file="/home/vinicius/ros2_ws/src/image_processor/masks_images/output.png")

