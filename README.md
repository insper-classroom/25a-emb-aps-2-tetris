# RP2040 FreeRTOS com OLED1

## Controle para o jogo Tetris

**Jogo:** TETRIS

---

### Ideia do Controle:

O controle consiste em:
- Dois botões rotativos:
  - Um para rotação da peça;
  - Outro para movimentação horizontal (esquerda e direita). Ao pressionar o botão de movimentação, a peça desce.
- Um botão para guardar a peça;
- Um botão de ligar/desligar;
- Um display para mostrar a próxima peça (ainda não está definido essa parte).

---

### Inputs e Outputs:

**Inputs:**
- Encoder rotativo 1 (rotação): Controla a rotação da peça;
- Encoder rotativo 2 (movimentação): Controla a movimentação horizontal e descida;
- Botão de guardar peça: Guarda a peça;
- Botão liga/desliga: Liga e desliga o controle.

**Outputs:**
- Display OLED: Mostra a próxima peça;
- LED indicador: Mostra se o controle está ligado.

---

### Protocolo:
- **USB:** Para detectar o controle no PC;
- **I2C:** Para comunicação com o display;
- **GPIO com interrupção:** Para os encoders e botões.

---

### Diagrama de Blocos:
![Diagrama_controle_Tetris drawio](https://github.com/user-attachments/assets/4867729b-e4f4-4f50-b030-57bf39ea9b91)

---

### Imagem do Controle:
![13cce9ca-3631-44f2-959a-bb348d3cabde](https://github.com/user-attachments/assets/e8a93acd-d167-496d-8cbe-7be8dae2b846)



