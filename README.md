# GA-System

O avanço tecnológico fez com que cada vez mais consumidores aderissem às compras por meios eletrônicos e, por consequência, a demanda por armazéns de distribuição mais eficientes aumentasse. As novas tecnologias também permitiram que os robôs móveis deixassem de ser apenas objeto de estudo, e se tornassem um elemento essencial em diversas áreas, incluindo o ramo da logística, pois os robôs são menos susceptíveis a erros, o que pode aumentar a eficiência dos armazéns, e seu custo operacional não cresce com o tempo. Porém, para que os robôs sejam eficientes em suas atividades, é necessário que o gerenciamento da distribuição e realização das tarefas executadas por eles seja otimizado.

A alocação de tarefas pode utilizar diversas técnicas, os métodos leilão e Algoritmo Genético (AG) estão entre as mais utilizadas. Contudo, as abordagens já desenvolvidas normalmente não atendem à restrições de prazo, energia e capacidades de carga dos robôs simultaneamente. Portanto, esta pesquisa desenvolveu uma arquitetura que emprega o AG utilizando Modelo de Ilhas, para alocação de tarefas em sistemas multi-robóticos, que atende a esses tipos de restrições ao mesmo tempo.

O sistema desenvolvido utiliza os pacotes de navegação e comunicação do ROS. Assim, qualquer conjunto de robôs que utilize uma versão deste sistema operacional pode utilizar a arquitetura desenvolvida.

As avaliações do sistema mostram que ele é capaz de alocar mais tarefas que o AG padrão quando utilizando a mesma heurística. Além disso, a abordagem desenvolvida encontra soluções onde a estimativa de consumo energético é menor que o esperado e atende restrições de carga de transporte, bateria e prazo. Também demonstraram que o sistema é funcional na coordenação de um grupo de robôs em um ambiente simulado.

Por fim, os resultados indicam que em um ambiente real de média escala pode-se utilizar o sistema. Para ambientes maiores, a obtenção de soluções otimizadas demanda um tempo para alocar tarefas que cresce exponencialmente com o número delas.
  
## Simulação

Pare realizar a simulação foi construído um ambiente de aproximadamente 50mx30m utilizando as prateleiras do pacote small-warehouse da Amazon: https://github.com/aws-robotics/aws-robomaker-small-warehouse-world 

O modelo dos robôs utilizados na simulação foi adaptado do repositório: https://github.com/RafBerkvens/ua_ros_p3dx 

Para executar a simulação, é necessário que o usuário clone este repositório no seu workspace do ROS, compile o sistema e tenha o banco de dados instalado e configurado com as tabelas do sistema. Com os requisitos atendidos basta executar os seguintes comandos na ordem em que aparecem: 

### Para rodar o ambiente
roslaunch p3dx_simulation sim_alan_warehouse.launch


### Para rodar o sistema interno dos robôs
roslaunch p3dx_simulation sim_system_client.launch

### Para rodar o coordenador
rosrun system_server system_server_node

### [p3dx_control](https://github.com/alankc/GA-System/tree/master/p3dx_control)
### [p3dx_description](https://github.com/alankc/GA-System/tree/master/p3dx_description)
### [p3dx_gazebo](https://github.com/alankc/GA-System/tree/master/p3dx_gazebo)
### [p3dx_navigation](https://github.com/alankc/GA-System/tree/master/p3dx_navigation)
### [p3dx_simulation](https://github.com/alankc/GA-System/tree/master/p3dx_simulation)

## Sistema interno aos robôs
No link a seguir estão os arquivos do sistema interno ao robô:
### [system_client](https://github.com/alankc/GA-System/tree/master/system_client)

## Sistema de coordenação e alocação de tarefas
No link a seguir estão os arquivos do sistema de coordenação e alocação de tarefas:
### [system_server](https://github.com/alankc/GA-System/tree/master/system_server)

## Banco de dados
O banco de dados utiliza o MySQL e foi modelado utilizando o MySQL Workbench.
No link a seguir estão os arquivos do banco de dados:
### [database](https://github.com/alankc/GA-System/tree/master/database)

## Resultados
Foram realizados dois tipos de experimentos: um para avaliar o alocador de tarefas de forma independente e outro para avaliar o sistema de coordenação e o funcionamento do sistema como um todo em simulação.

No link a seguir estão os dados extraídos dos experimentos:
### [results](https://github.com/alankc/GA-System/tree/master/results)
