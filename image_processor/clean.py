import os
import glob

def clear_directory(directory_path):
    files = glob.glob(os.path.join(directory_path, '*'))
    for f in files:
        try:
            os.remove(f)
            print(f"Removido: {f}")
        except Exception as e:
            print(f"Erro ao remover {f}: {e}")

directories = [
    "/home/vinicius/ros2_ws/src/image_processor/masks_images/vis",
    "/home/vinicius/ros2_ws/src/image_processor/masks_images/pred",
    "/home/vinicius/ros2_ws/src/image_processor/processed_images"
]

for directory in directories:
    clear_directory(directory)
    print(f"Todos os arquivos em {directory} foram removidos.")
