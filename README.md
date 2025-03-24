# RP2040 freertos with OLED1

# Controle para o jogo Tetris

Jogo: TETRIS;
Ideia do controle:
  O controle consiste em:
  - Dois botões rotativos onde, um é para a a rotação da peça e o outro para movimentação horizontal (esqueda e direita) e, ao pertar o botão de movimentação, ele desce a peça;
  - Um botão para guardar a peça;
  - Um botão de ligar;
  - Um display para mostrar a proxima peça (ainda não está definido essa parte);
Inputs e Outputs:
  Inputs:
  - Encorde rotativo 1 (rotação): Controla a rotação da peça;
  - Encorde rotativo 2 (movimentação): Controla a movimentação horizontal e descida;
  - Botão de guardar peça: Guarda a peça;
  - Botão liga/desliga: Ligar e desligar o controle;
  Outputs:
  - Display OLED: Mostra a proxima peça;
  - LED indicador: Mostra se o controle está ligado;
Protocolo:
  - USB: Para detectar o controle no PC;
  - I2C: Para comunicação com o display;
  - GPIO com interrupção: Para os encorders e botões;
Diagrama de blocos:
  - 
