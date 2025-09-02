import pygame
import sys
from is_to_ros import SkeletonPosition

# Inicializa o Pygame e o canal
pygame.init()
position = SkeletonPosition("amqp://10.20.5.3:30000", "ros.skeleton_pose")
# Dimensões da tela
WIDTH, HEIGHT = 600, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Grid 9x9 com Controle WASD")

# Cores
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Fonte
font_size = 40
font = pygame.font.SysFont(None, font_size)

# Número de linhas e colunas
ROWS, COLS = 9, 9
spacing_x = WIDTH // (COLS + 1)
spacing_y = HEIGHT // (ROWS + 1)

# Centro do grid em pixels
center_x = WIDTH // 2
center_y = HEIGHT // 2

# Escala: cada passo entre X equivale a 0.5m
meters_per_step = 0.5

# Pega input do usuário no terminal
try:
    user_input = input("Digite a posição inicial em metros (ex: 1.0 0.5): ")
    pos_x_m, pos_y_m = map(float, user_input.split())
except:
    print("Entrada inválida. Use o formato: x y (em metros)")
    pygame.quit()
    sys.exit()

# Função para converter metros -> pixels
def meters_to_pixels(x_m, y_m):
    px = center_x + (x_m / meters_per_step) * spacing_x
    py = center_y - (y_m / meters_per_step) * spacing_y  # Inverte Y
    return int(px), int(py)

# Imprime posição inicial
print(f"Posição atual: x={pos_x_m:.2f} m, y={pos_y_m:.2f} m")
position.send_to(msg= [pos_x_m, pos_y_m, 0])

gesture = 0
# Loop principal
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Captura teclas pressionadas
    keys = pygame.key.get_pressed()
    moved = False
    
    if keys[pygame.K_w]:  # Para cima
        pos_y_m += meters_per_step
        moved = True
    if keys[pygame.K_s]:  # Para baixo
        pos_y_m -= meters_per_step
        moved = True
    if keys[pygame.K_a]:  # Para esquerda
        pos_x_m -= meters_per_step
        moved = True
    if keys[pygame.K_d]:  # Para direita
        pos_x_m += meters_per_step
        moved = True

    # Gestos a serem identificados
    # P = Stop; V = Come to me; F = Follow me
    if keys[pygame.K_p]: 
        gesture = 1
    if keys[pygame.K_v]:  
        gesture = 2
    if keys[pygame.K_f]:  
        gesture = 3
    
    if moved or gesture != 0:
        pygame.time.wait(150)  
        position.send_to(msg=[pos_x_m, pos_y_m, gesture])
        if gesture == 1 or gesture == 2:
            gesture = 0

    # Preenche a tela de branco
    screen.fill(WHITE)

    # Desenha os X
    for row in range(ROWS):
        for col in range(COLS):
            x = (col + 1) * spacing_x
            y = (row + 1) * spacing_y

            if row == 4 and col == 4:
                text = font.render("X", True, RED)
            else:
                text = font.render("X", True, BLACK)

            rect = text.get_rect(center=(x, y))
            screen.blit(text, rect)

    # Converte posição da bolinha para pixels e desenha
    pixel_x, pixel_y = meters_to_pixels(pos_x_m, pos_y_m)
    pygame.draw.circle(screen, BLUE, (pixel_x, pixel_y), 10)

    pygame.display.flip()
