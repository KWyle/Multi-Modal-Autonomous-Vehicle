import pygame
import sys

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 800, 400
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car Speed Simulation")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Speed parameters
speed = 0.0
max_speed = 200.0
acceleration_rate = 0.2  # Speed increase per frame when W is held
deceleration_rate = 0.1  # Natural deceleration when no keys pressed
brake_rate = 1        # Deceleration when S is pressed

# Clock for controlling frame rate
clock = pygame.time.Clock()
FPS = 60

# Font for displaying speed
font = pygame.font.Font(None, 48)

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Get pressed keys
    keys = pygame.key.get_pressed()
    
    # Update speed based on input
    if keys[pygame.K_w]:
        # Accelerate
        speed = min(speed + acceleration_rate, max_speed)
    elif keys[pygame.K_s]:
        # Brake (decelerate faster)
        speed = max(speed - brake_rate, 0)
    else:
        # Natural deceleration
        speed = max(speed - deceleration_rate, 0)
    
    # Clear screen
    screen.fill(BLACK)
    
    # Draw speed bar
    bar_width = int((speed / max_speed) * (WIDTH - 100))
    bar_color = GREEN if not keys[pygame.K_s] else RED
    pygame.draw.rect(screen, bar_color, (50, HEIGHT // 2 - 25, bar_width, 50))
    pygame.draw.rect(screen, WHITE, (50, HEIGHT // 2 - 25, WIDTH - 100, 50), 2)
    
    # Display speed text
    speed_text = font.render(f"Speed: {speed:.1f} Mph", True, WHITE)
    screen.blit(speed_text, (WIDTH // 2 - speed_text.get_width() // 2, HEIGHT // 2 + 60))
    
    # Display controls
    controls_font = pygame.font.Font(None, 24)
    controls_text = controls_font.render("W: Accelerate | S: Brake | Release: Coast", True, WHITE)
    screen.blit(controls_text, (WIDTH // 2 - controls_text.get_width() // 2, HEIGHT - 40))
    
    # Update display
    pygame.display.flip()
    
    print(speed)
    
    # Control frame rate
    clock.tick(FPS)

pygame.quit()
sys.exit()