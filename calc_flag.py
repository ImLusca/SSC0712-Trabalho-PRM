import math

# codigo a ser incluído em ~/prm/controle_robo.py

self.flag_pos = None

def calc_flag_pos(self, x_pix, bbox_h, bot_x, bot_y, bot_ang):
    # Função para calcular posição da flag uma vez que encontrada
    #   
    #   x_pix   ->  Centralização horrizontal da bandeira na imagem;
    #   bbox_h  ->  Altura da Bounding Box da bandeira;
    #   bot_x   ->  Posição em x do robô;
    #   bot_y   ->  Posição em y do robô;
    #   bot_ang ->  Ângulo do robô.

    # Ângulos em radianos

     
    # Dados da câmera
    img_w = 320 # altura
    img_h = 240 # largura
    hfov = 1.57 # campo de visão horriontal
    vfov = hfov * img_h / img_w  # campo de visão vertical

    # Altura real da bandeira (mastro)
    flag_h = 0.4

    # Altura da bounding box em pixels
    if bbox_h <= 0:
        return None  # Evita divisão por zero


    # Estimar distância usando semelhança de triângulos
    focal_length_pixels = (img_h / 2) / math.tan(vfov / 2)
    distance = (flag_h * focal_length_pixels) / bbox_h

    # Ângulo horizontal relativo à câmera (em rad)
    ang_per_pix = hfov / img_w
    delta_a = (x_pix - img_w / 2) * ang_per_pix

    # Ângulo absoluto no mapa
    abs_ang = bot_ang + delta_a

    # Coordenadas da bandeira
    flag_x = bot_x + distance * math.cos(abs_ang)
    flag_y = bot_y + distance * math.sin(abs_ang)

    self.flag = [flag_x, flag_y]
    ###
    ###     INCLUIR - Update do mapa de bandeira
    ###

    return flag_x, flag_y, distance