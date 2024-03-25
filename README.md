# Robot@Home

A liga RoboCup @Home visa desenvolver tecnologia de serviço e robô de assistência para futuras aplicações domésticas pessoais. Os robôs precisam realizar tarefas e testes dentro de uma casa simulada, um ambiente realista e não padronizado [[CBRobotica]].

O foco está nos seguintes domínios: Interação e Cooperação Humano-Robô, Navegação e Mapeamento em ambientes dinâmicos, Visão Computacional e Reconhecimento de Objetos sob condições de luz natural, Manipulação de Objetos, Comportamentos Adaptativos, Integração de Comportamento, Inteligência Ambiental, Padronização e integração de sistemas [[CBRobotica]].


# Habilidades desejadas pela Regras do Campeonato
- Navegação em ambientes dinâmicos
- Calibração e configuração rápidas e fáceis (o objetivo final é ter um robô instalado e funcionando ao sair da caixa)
- Reconhecimento de objetos
- Manipulação de objetos
- Detecção e reconhecimento de humanos
- Interação natural entre homem e robô
- Reconhecimento de fala
- Reconhecimento de gestos
- Aplicações de robôs (@Home visa aplicações de robôs na vida diária)
- Inteligência ambiental, como comunicação com dispositivos próximos, recuperação de informações informações da internet, etc.


# Testes do Robô na Competição

## Estágio 1 - Navegação e siga-me

O robô deve navegar por uma variedade de waypoints e, quando solicitado, seguir um operador a uma distância segura, evitando bater em objetos ou quaisquer obstáculos no caminho. Ao final da prova o robô deverá sair da arena [[regras]].

**Objetivo principal**: O objetivo principal deste teste é avaliar se o robô é capaz de navegar no cenário e seguir uma pessoa [[regras]].


## Estágio 1 - Reconhecimento de fala e detecção de áudio

O robô deve responder a um conjunto de perguntas a um operador na primeira tentativa, sem pedir confirmação. O operador não tem permissão para se mover em direção ao robô ou gritar para ele. O operador não tem permissão para se mover em direção ao robô ou gritar para ele. As questões estarão relacionadas com o contexto da competição Robocup@Home [[regras]].

**Objetivo principal**: O robô deve ser capaz de reconhecer e responder a um conjunto de perguntas sem pedir confirmação [[regras]].


## Estágio 1 - Reconhecimento de Pessoas

Um Operador é apresentado ao robô, que precisa aprender como é a aparência do Operador. Depois que o robô tiver coletado informações suficientes sobre o Operador, o Operador se mistura à multidão e o robô precisa encontrá-lo. Depois que o robô encontrar seu Operador, ele deverá explicar como deve fornecer informações sobre o Operador [[regras]].

*Objetivo principal*: O robô deve identificar o Operador dentro de uma multidão e declarar informações sobre o Operador e a multidão [[regras]].

*Objetivo opcional*: Identificar a operadora pelo seu nome [[regras]].


## Estágio 1 - Manipulação e reconhecimento de objetos

O robô deve chegar a um local de coleta (estante com prateleiras ou mesa) onde existam 10 objetos. O robô deve então identificar e agarrar 5 desses objetos e colocá-los em um local de entrega (outra prateleira ou outra mesa) [[regras]].

**Objetivo principal**: O robô deve identificar, agarrar e posicionar corretamente diversos objetos em diferentes alturas ou posições [[regras]].

**Objetivo opcional**: Encontrar um objeto oculto ou obstruído [[regras]].


# Arquitetura do Robo

![alt text](docs/architecture.png)


# Como começar

Configure as variaveis de ambiente do ROS Humble
- source 0-init.sh 


# Gerenciamento do Projeto

Documentação sobre o andamento do projeto [aqui](./0-gerenciamento/readme.md)



[CBRobotica]: https://www.cbrobotica.org/index.php/categorias/
[regras]: https://github.com/RoboCupAtHomeLatinAmerica/RuleBook/blob/2023_draft/build/Rulebook.pdf