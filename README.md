# Controle de Nível e Vazão de Água

**Controle de Nível e Vazão de Água** é um sistema experimental de automação desenvolvido para monitoramento e controle de um processo hidráulico com bomba, sensor ultrassônico de nível e sensor de vazão. O projeto integra firmware embarcado em Arduino com interfaces web para operação em **malha aberta** e **malha fechada**, permitindo supervisão, calibração, aquisição de dados e ajuste de parâmetros de controle.

## Visão geral

O projeto foi concebido como uma plataforma de experimentação para sistemas de controle aplicados a processos hidráulicos. Sua proposta é permitir que variáveis físicas, como **nível** e **vazão**, sejam monitoradas em tempo real, ao mesmo tempo em que estratégias de controle possam ser avaliadas em diferentes modos de operação.

A arquitetura adotada combina:

- aquisição de dados via sensores;
- atuação sobre bomba por sinal PWM;
- comunicação serial com interface web;
- operação em malha aberta e malha fechada;
- visualização gráfica e coleta de dados experimentais.

## Objetivo do software

O objetivo do sistema é implementar uma plataforma didática e experimental para controle de nível e vazão de água, permitindo o estudo prático de:

- aquisição de sinais de sensores;
- atuação em sistemas físicos por PWM;
- calibração de processos;
- modelagem empírica por curva cúbica;
- controle PID em malha fechada;
- supervisão e registro de dados em tempo real.

## Contribuição científica

A principal contribuição do projeto está na integração, em uma única solução, de:

1. **instrumentação física de baixo custo**;
2. **controle embarcado em tempo real**;
3. **interfaces web interativas para supervisão e operação**;
4. **alternância entre malha aberta e malha fechada**;
5. **coleta automatizada de dados experimentais**.

Do ponto de vista metodológico, o sistema não se limita à automação simples de uma bomba. Ele estrutura um ambiente de experimentação que permite calibração de atuadores, avaliação de resposta dinâmica do sistema e estudo comparativo entre estratégias de controle.

## Problema abordado

Processos hidráulicos com reservatórios demandam monitoramento contínuo do nível de fluido e, em muitos casos, controle da vazão ou do nível desejado. Em ambientes didáticos e de prototipagem, é importante dispor de uma plataforma que permita testar estratégias de controle de forma clara, visual e reproduzível.

Este projeto foi desenvolvido para atender a esse cenário, oferecendo uma solução que integra sensores, atuador, firmware de controle e interface de supervisão.

## Arquitetura do sistema

O sistema é composto por três partes principais:

### 1. Firmware embarcado

O arquivo `controle_final.ino` implementa a lógica principal do sistema em Arduino, sendo responsável por:

- leitura do sensor ultrassônico HC-SR04;
- leitura do sensor de vazão YF-S201;
- cálculo das variáveis do processo;
- controle da bomba por PWM;
- execução das estratégias de malha aberta e malha fechada;
- comunicação serial com a interface web.

### 2. Interface em malha aberta

O arquivo `malha_aberta.html` implementa a interface para operação em malha aberta, permitindo:

- conexão serial com o Arduino;
- acionamento da bomba;
- definição de vazão desejada;
- calibração da curva cúbica da bomba;
- calibração da distância do sensor;
- visualização de nível, vazão e duty cycle;
- coleta e exportação de dados em CSV.

### 3. Interface em malha fechada

O arquivo `malha_fechada.html` implementa a interface para operação em malha fechada, permitindo:

- conexão serial com o Arduino;
- definição do setpoint de nível;
- ajuste dos parâmetros PID;
- visualização de nível, vazão e duty cycle;
- coleta de dados experimentais;
- monitoramento da resposta temporal do sistema.

## Sensores e atuadores

O sistema utiliza os seguintes elementos físicos:

### Sensor ultrassônico HC-SR04
Responsável pela medição do nível de água no reservatório.

### Sensor de vazão YF-S201
Responsável pela medição da vazão instantânea.

### Bomba d’água com acionamento PWM
Responsável pela variável manipulada do processo.

## Conexões físicas

### Sensor ultrassônico HC-SR04
- VCC -> 5V do Arduino
- GND -> GND do Arduino
- TRIG -> pino 7
- ECHO -> pino 6

### Sensor de vazão YF-S201
- VCC (vermelho) -> 5V do Arduino
- GND (preto) -> GND do Arduino
- Sinal (amarelo) -> pino 49

### Bomba / driver
- PWM -> pino 3
- GND -> GND do Arduino

## Modos de operação

O sistema possui dois modos principais de operação:

### Malha aberta

Na malha aberta, o usuário pode:

- ligar ou desligar a bomba manualmente;
- definir um setpoint de vazão;
- aplicar uma curva de calibração cúbica para mapear vazão desejada em duty cycle.

Nesse modo, o duty cycle é estimado pela equação:

$$
u(q) = aq^3 + bq^2 + cq + d
$$

em que:

- \(q\) é a vazão desejada;
- \(a, b, c, d\) são coeficientes de calibração;
- \(u(q)\) é o duty cycle em porcentagem.

### Malha fechada

Na malha fechada, o sistema realiza controle PID com base no nível medido. O erro de controle é dado por:

$$
e(t) = h_{ref}(t) - h(t)
$$

em que:

- \(h_{ref}(t)\) é o nível de referência;
- \(h(t)\) é o nível medido.

A ação de controle PID pode ser descrita por:

$$
u(t) = K_c \left[e(t) + \frac{1}{T_i}\int e(t)\,dt + T_d\frac{de(t)}{dt}\right]
$$

Na implementação discreta utilizada no firmware, os termos são calculados como:

$$
P_k = K_c e_k
$$

$$
I_k = I_{k-1} + \frac{K_c}{T_i} e_k \Delta t
$$

$$
D_k = K_c T_d \frac{e_k - e_{k-1}}{\Delta t}
$$

e a ação total é:

$$
PID_k = P_k + I_k + D_k
$$

Depois disso, o valor é saturado no intervalo de 0 a 100 e convertido em PWM:

$$
PWM = \frac{PID_k}{100} \cdot 255
$$

## Modelagem das variáveis

### Nível

O nível é estimado a partir da diferença entre a distância calibrada do sensor ao fundo do reservatório e a distância medida pelo ultrassom:

$$
h = d_{cal} - d_{med}
$$

Na implementação, a conversão aproximada da leitura é feita por:

$$
d_{med} = \frac{\text{medicao}}{58.8}
$$

### Vazão

A vazão é calculada a partir da frequência medida no sensor de fluxo:

$$
Q = \frac{f}{7.5}
$$

em que:

- \(Q\) é a vazão em L/min;
- \(f\) é a frequência estimada do sensor.

## Funcionalidades

O sistema oferece as seguintes funcionalidades:

- monitoramento de nível em tempo real;
- monitoramento de vazão em tempo real;
- acionamento manual da bomba;
- controle em malha aberta;
- controle PID em malha fechada;
- ajuste de setpoint de vazão;
- ajuste de setpoint de nível;
- ajuste de \(K_c\), \(T_i\) e \(T_d\);
- calibração da curva de atuação;
- calibração da distância do sensor;
- visualização gráfica da tendência temporal;
- exportação de dados em CSV;
- interface responsiva em navegador.

## Estrutura do projeto

```txt
Controle_Nivel_Vazao/
├── .gitignore
├── controle_final.ino
├── malha_aberta.html
└── malha_fechada.html
```

## Tecnologias utilizadas

### Firmware
- Arduino / C++
- NewPing
- FreqMeasure

### Interface
- HTML
- CSS
- JavaScript
- Chart.js
- Web Serial API

## Instalação

### 1. Preparação do firmware

Abrir o arquivo `controle_final.ino` na Arduino IDE.

Instalar as bibliotecas necessárias:

- `NewPing`
- `FreqMeasure`

Selecionar a placa e a porta corretas e fazer o upload do código para o Arduino.

### 2. Execução da interface

Como o projeto utiliza páginas HTML puras, basta abrir no navegador:

- `malha_aberta.html` para operação em malha aberta;
- `malha_fechada.html` para operação em malha fechada.

> Recomenda-se utilizar navegador com suporte à **Web Serial API**, como Google Chrome ou Microsoft Edge.

## Exemplo de uso

### Malha aberta

1. Abrir `malha_aberta.html`.
2. Conectar o Arduino pela interface.
3. Acionar a bomba manualmente ou definir a vazão desejada.
4. Observar os gráficos e os indicadores de nível, vazão e duty cycle.
5. Coletar os dados experimentais e exportar em CSV, se desejado.

### Malha fechada

1. Abrir `malha_fechada.html`.
2. Conectar o Arduino.
3. Definir o nível de referência.
4. Ajustar os parâmetros \(K_c\), \(T_i\) e \(T_d\).
5. Observar a resposta do sistema no gráfico de tendência.
6. Registrar os dados experimentais para análise posterior.

## Comandos seriais suportados

O firmware aceita os seguintes comandos pela porta serial:

### Comandos gerais
- `Modo:Aberta`
- `Modo:Fechada`
- `Calibragem:XX.XX`

### Comandos da malha aberta
- `Bomba:Ligada`
- `Bomba:Desligada`
- `Vazao:XX.XX`
- `Calibrar:a,b,c,d`

### Comandos da malha fechada
- `Nivel:XX.XX`
- `Kc:XX.XX`
- `Ti:XX.XX`
- `Td:XX.XX`

## Saída serial

O sistema envia dados no formato:

```txt
Bomba:Ligada|Nivel:12.34|Vazao:3.21
|Duty:65.0
```

Esses dados são utilizados pela interface para atualização dos indicadores e gráficos.

## Segurança operacional

O firmware possui uma proteção simples para desligamento da bomba quando o nível ultrapassa o limite máximo definido. Nessa condição, o PWM é zerado temporariamente para evitar transbordamento.

## Aplicações

O projeto pode ser aplicado em:

- bancadas didáticas de controle de processos;
- ensino de instrumentação e automação;
- estudos de malha aberta e malha fechada;
- experimentos com controle PID;
- aquisição de dados de processos hidráulicos;
- prototipagem de sistemas supervisórios de baixo custo.

## Limitações

Entre as limitações atuais, destacam-se:

- calibração dependente das características físicas do sistema;
- sensibilidade do sensor ultrassônico a ruídos e reflexões;
- necessidade de ajuste manual dos parâmetros PID;
- dependência de navegador com suporte à Web Serial API;
- ausência de persistência em banco de dados.

## Trabalhos futuros

Possíveis extensões do projeto incluem:

- armazenamento histórico em banco de dados;
- integração com dashboard web remoto;
- inclusão de identificação de sistemas;
- sintonia automática de PID;
- envio de dados para nuvem;
- geração automática de relatórios experimentais.

## Autores

- **Nicole Maia Argondizzi**
- **Isaac Miranda Camargos**

## Como citar

```txt
ARGONDIZZI, Nicole Maia; CAMARGOS, Isaac Miranda. Controle de Nível e Vazão de Água:
sistema experimental com operação em malha aberta e malha fechada para monitoramento,
aquisição de dados e controle PID. GitHub.
```

## Licença

```txt
MIT License
```
