# tese-controlador


------------------- PARA CORRER O PROGRAMA ----------------------------

estado 1) normal
 . correr ficheiro "run_simulation"  

estado 2) depois de alteração ao código:

. correr simulink (não sei a razão, mas é necessário realizar este passo para acontecerem diferenças nos resultados)
. correr ficheiro "run_simulation" 


------------------- CENÁRIOS ----------------------------

.Cenário 1- lemniscade (colocar scenario = 1 na função "conditions")
.Cenário 2- manobra de captura (colocar scenario = 2 na função "conditions")


--------------------Ficheiros, Funções e variáveis--------------------------------

"run_animation": para correr a simulação
.anima: função para animação em 3D da posição

"conditions": live function para se puder alterar condições a meio da função; tmabém define as condições iniciais da simulação
.initialization: função de initialização




