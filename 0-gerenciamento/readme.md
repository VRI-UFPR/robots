# Objetivo Primario
- construir um robo de serviço e concorrer no campeonato
- ajudar aa comunidade Brasero(robotica) a crescer

# Motivação
- ter um dominio sobre SLAM, processamento de imagens, IA, sistemas distribuidos, processamento de Linguagem Natural. Pois estes estão em alta e sao importante tecnologias para o desenvolvimento economico brasileiro dentro da atual revolucao industrial.
- conseguir atrair e ensinar mais pessoas a robotica e programacao. A ideia é a robotica se tornar mais popular e menos elistista.
- ajudar aos participantes deste projeto, ter um projeto bacana no github e assim poder coloca-los em seus curriculos.

# Cenário

A revolução industrial está ocorrendo a passos rapidos. Levando em conta que a robotica, sistemas distribuidos, IA e comunicacao sem fio sao a vanguarda dessa revolucao. Neste censario, O Brasero e o VRI  estao bem colocados visto que sao a elite da robotica brasileira no momento. E se tivermos sucesso, ou seja, conseguimos fazer um codigo bem feito, facil de usar e compartilhar, e outros passarem a criar seus robos baseados nos codigos do brasero ou do VRI. Entao isso ajudara a avançar a tecnologia brasileira  e nos tornarmos a referencia brasileira na area em poucos anos.




# Tarefa 1 - Movimentação do robo

Conseguir compilar o libaria com ROS 2 para movimentar o robo. 
A tarefa é concluída quando tiver um codigo para movimentar o robo atraves do teclado.

Pode-se fazer em um repositorio separado, e depois integrar com o repositorio do robo

O que se pode aprender ou ser interessante:
 - aprender o basico sobre ROS
 - quais os principais topicos para uma base de robo funcionar

Previsao de termino: ???



# Linha de Trabalho - Hardware

**Tarefa HW-1 - fazer uma bateria de 19v para a Jetson**

Preparar uma bateria de 19v usando as baterias de litio que nós jah temos. Conversar com o Prof. Todt para definir como juntar as baterias.

O que se pode aprender:
 - como recarregar as baterias de litio
 - talvez imprimir um case das baterias na impressora 3D

**Tarefa HW-2 - Botao vermelho de parada**

O campeonato exige que o robo tenha um botao vermelho que pare o robo. Nao sei se pode fazer isso atraves de software ou terá que mexer na parte eletrica do robo. Isso precisa ser estudado e conversado com o Prof. Todt.

O que se pode aprender ou ser interessante:
 - aprender um pouco de eletronica




# Linha de Trabalho - Interface Humano-Computador (IHC)

**Tarefa permanente : Compreender o funcionamento do seq2seq**

Compreender o funcionamento do seq2seq (já está na constantine na minha home, o modelo atual utiliza atenção)

Ao mesmo tempo de estudar e aprender a usar uma seq2seq, é importante ir criando uma documentação para ficar mais facil para outras pessoas aprenderem tbem sobre esta tecnologia. Levando em conta que os transformers tiveram um grande impacto sobre tradutores e estão em alta para processamento de linguas naturais.

Trabalhar nesta tarefa continuamente, e verificar se os iniciantes no assunto gostam da didatica do material

Repositorio: https://github.com/VRI-UFPR/Como-aprender-Transformers

**Tarefa IHC-1 : Treinar uma simples IA seq2seq [OK]**

Fazer uma IA que dado uma ordem em lingua natural etnica converter para uma lista de orders limitadas e de preferencia sem ambiguidades. Para isso será necessario buscar uma base dados ou procurar uma, seguindo o padrao para treinar a IA.

Exemplo:
  - Move 45 meters -> Move (45 meters)
  - Move forward for 45 meters -> Move (45 meters)

Repositorio: !!!Está na constantine, ver onde colocar este codigo!!!

**Tarefa IHC-2 : Fazer uma lista de API para IHC e IA**

Uma opção mais fácil e prática para IHC será achar uma boa API, que faça isso. Havia pesquisado uma opção chamada “Gorillas”


**Decisao ICH-3 : Escolher entre a API e a nossa IA**




# Linha de Trabalho - Visão Computacional (VC)

**Tarefa VC-1: Criar um dataset e treinar uma nova rede YOLO**
Atualmente, a rede YOLO está está treinada para detectar objetos mais gerais pois foi treinada por outras pessoas. É preciso criar um dataset específico da competição.

**Tarefa VC-2: Melhorar o reconhecimento facial**
Atualmente temos código bom para detectar rostos, mas o método de reconhecimento ainda está meio ruim, não detecta sempre o rosto que queremos.

**Tarefa VC-3: Pensar em um código de tracking**
Os códigos para detecção de objetos e rostos lê cada frame separadamente. Talvez seja uma opção alguns métodos de tracking, para garantir que o robô não perca o seu alvo. 

**Tarefa VC-4: Sinais do instrutor**
As vezes o instrutor vai dar um sinal para que o robô faça algo, esse sinal poderá ser idenficado com visão? Acredito que sim., como um sinal com as mãos. 

**Tarefa VC-5: Etapa do personal recognition**
A tarefa do personal recognition requer que o robô conte quantas pessoas está no ambiente e andar até uma específica, é bom pensar em como implementar essa parte do robô, que junta várias áreas.



# Linha de Trabalho - Localização e Mapeamento (SLAM)

Link do repositório onde eu estou trabalhando: https://github.com/VRI-UFPR/VRI-stereo-vo

**Tarefa SLAM-1: Implementar os publishers para os sensores [OK]**
* O pacote drivers no repositório já contém um publisher para a câmera IMX219 e um para a IMU BNO08x, que são os sensores que pretendia usar no módulo de VIO.

**Tarefa SLAM-2: Escolher e implementar os algoritmos para o VO**
* A arquitetura do sistema de VO está dividida em 4 módulos principais separados como ROS services para poderem ser executados em paralelo: estimador de profundidade (entre dois frames stereo), extrator de features, correspondência de features (entre dois frames consecutivos) e a estimação de movimento (a partir da correspondência de features entre os dois frames). Ainda falta decidir e implementar os algoritmos de cada módulo. Existem algumas opções disponíveis no próprio OpenCV e também há a possibilidade de usar modelos de machine learning.

**Tarefa SLAM-3: Implementar o algoritmo para a fusão dos dados da IMU com o VO**
* Depois de obter a estimação de pose pelo VO, será necessário fundir com a estimação da IMU. As opções mais populares que encontrei aqui são um filtro de kalman (MSCKF) ou um problema de otimização baseado em um grafo de poses. A princípio pensei em seguir com o Kalman.

**Tarefa SLAM-4: Integração do módulo de VIO com o robô**
* Considerando que o módulo ficara separado do resto do sistema do robô, teria que pensar na forma como a estimação de pose gerada por ele seria enviada para o resto do sistema. A mais simples é conectar na mesma rede local e usar a própria estrutura do ROS, mas podem haver problemas de conexão. Outra alternativa que pensei em experimentar seria por USB.

**Tarefa SLAM-5: Design da estrutura do módulo e circuito**
* É bem importante que a posição entre as duas câmeras no setup stereo seja conhecida, assim como a posição da IMU para o resto do setup. Assim, seria feito uma "caixa" com impressão 3D para acomodar os componentes (Jetson Nano, IMU e duas câmeras). A conexão da IMU para a Jetson seria feito pelo GPIO com I2C e a alimentação (3.3v) também. Ainda preciso decidir como alimentar a Jetson e queria ver a opnião do professor sobre essa parte


# Linha de Trabalho - Ensino e suporte ao Brasero

**Tarefa EDU-1 - Roadmap sobre projeto para iniciantes**

O projeto eh complexo e exige dos participantes um certo conhecimento. Pensar em quais conhecimentos sao necessarios e depois fazer uma lista de videos e tutoriais sobre assunto. Tentar achar bons videos no youtube sobre cada assunto.

 De modo, que ao final, a pessoa novata tenha capacidade de desenvolver o projeto

Isso será bastante util para novatos e futuros novatos. E estah alinhado aa motivacao de popularizar a robotica

**Tarefa EDU-2 - Suporte ao Brasero**

Foi aprovado na reuniao fazer um repositorio com as informacoes dos times, principalmente tutoriais. Isso tem como objetivo melhorar o compartilhamento de informacao entre os times e nao ficarmos trabalhando nos mesmos items. Entao podemos tentar coletar informacoes e organiza-los. E futuramente passar para o Fagner colocar no repositorio do Brasero.

Nós jah temos as analises do TDPs, o que é uma boa contribucao.



# Linha de Trabalho - Arquitetura e Engenharia de Software


**Tarefa Arq-1 - Mecanismo de compilacao e execucao do codigo no raspberry**

A ideia é fazer script para facilitar a compilacao do codigo do raspberry e sua execucao no raspberry.

Depende do termino da tarefa: movimentacao do robo, pois terá um codigo para testar

O que pode aprender ou ser interessante:
 - aprender um pouco como estruturar um sistema de deploy.
 - pode-se olhar se o ROS tem algo para fazer isso

Previsao: