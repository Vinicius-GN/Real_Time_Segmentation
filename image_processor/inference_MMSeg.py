#Possibilities of methods to receive the raw_images to process
# images = ["/home/vinicius/ros2_ws/src/image_processor/images_source/000000646.png","/home/vinicius/ros2_ws/src/image_processor/images_source/000000647.png"] # image1 can be a file path or a np.ndarray
# images = "/home/vinicius/ros2_ws/src/image_processor/images_source/"

import sys
import yaml
from mmseg.apis import MMSegInferencer

def load_config(config_path):
    with open(config_path, 'r') as file:
        return yaml.safe_load(file)

def main():
    #Caminho do arquivo de configuração
    config_path = '/home/vinicius/ros2_ws/src/image_processor/config/define_net.yaml'
    
    config = load_config(config_path)

    #Configurações do modelo
    model_config = config['model']['config_file']
    model_weights = config['model']['weights_file']

    # Configurações de inferência
    input_images_dir = config['output']['input_images_dir']
    output_dir = config['output']['out_dir']
    vis_dir = config['output']['vis_dir']
    pred_dir = config['output']['pred_dir']

    #Inicializa o inferenciador
    inferencer = MMSegInferencer(model=model_config, weights=model_weights)

    #Define as imagens para inferência
    #images = sys.argv[1] if len(sys.argv) > 1 else input_images_dir

    images = "/home/vinicius/ros2_ws/src/image_processor/images_source/000000000.png"

    #Realiza a inferência
    inferencer(images, out_dir=output_dir, img_out_dir="vis", pred_out_dir="pred", return_datasamples=True)

    #Possibilidade de melhoria: imprimir no terminal ou em um tópico que o processo desejado foi concluido. com isso, tornamos o feedback de processamento atuomático
    #Outra possibilidade é arrumar a integração com o mmsegmentation no root e ver se rosolve

if __name__ == "__main__":
    main()
