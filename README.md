# Projeto de robô autônomo de tração diferencial utilizando ROS 2 e micro-ROS

## Resumo
A robótica tem se consolidado como uma área essencial em diversas aplicações, desde a
automação industrial até soluções para o cotidiano, como assistentes pessoais. Com o avanço
da tecnologia, esses sistemas estão cada vez mais com uma maior autonomia, permitindo
a execução de tarefas cada vez mais complexas. Neste contexto, este trabalho teve como
objetivo desenvolver um robô móvel autônomo de tração diferencial, projetado para operar
em ambientes internos (indoors). O foco foi explorar e integrar as ferramentas ROS 2 e
micro-ROS, aprofundando o entendimento dessas tecnologias e demonstrando sua aplicação
em sistemas robóticos de baixo custo. O projeto começou com simulações no Gazebo
para validar as leis de controle e a cinemática do robô, culminando na implementação
prática de um protótipo equipado com sensores ultrassônicos, uma unidade de medição
inercial (IMU), encoders magnéticos, motores de corrente contínua e comunicação via
Wi-Fi utilizando um microcontrolador ESP32. Os resultados confirmaram a eficácia do
ROS 2 como uma ferramenta poderosa para o desenvolvimento e validação de robôs móveis,
permitindo o uso de pacotes da comunidade para acelerar o processo de implementação.
Contudo, foram observadas algumas limitações, como inconsistências nas leituras dos
sensores ultrassônicos, baixa autonomia das baterias e falhas ocasionais na comunicação
entre o notebook e o microcontrolador. Apesar desses desafios, o robô conseguiu realizar
tarefas de locomoção autônoma e desvio de obstáculos, atingindo os objetivos propostos.
Conclui-se que, embora o projeto tenha cumprido seus objetivos iniciais, melhorias podem
ser feitas. Para trabalhos futuros, recomenda-se o uso de sensores mais avançados, como o
RPLIDAR com varredura de 360°, para maior precisão na percepção do ambiente, além
da utilização do pacote Nav2 para navegação mais eficiente.
