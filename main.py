import function as fun
import numpy as np

#данные о времени
tau = 0.01
t_start = 0.0
t_end = 17.

#начальные данные
start_accsel = 0.
start_position = 0.0
end_position = 40.0
start_speed = 0.0

#Константы
I_y = 7.16914 * 1.0e-5
Tcmd = 10.
L = 0.17
k_b = 3.9865 * 1.0e-8

##при подборе важно было добится достижения заданного угла без колебаний. Что особенно важно для углов близких к переходным
k_p = np.asarray([ .01, 0.1 ])
k_i = np.asarray([ .00001, .0001 ])
k_d = np.asarray([ 2.65, 18.95 ])

ParametrsOfSystem = fun.parametrs(start_position, start_speed, start_accsel, L, k_b, I_y, Tcmd )
Control = fun.control_function( end_position, 0.0, k_p, k_i, k_d )

PosList, SpeedList, AccselList = fun.quad_sim(ParametrsOfSystem, Control, tau, t_end)
fun.plot_graph(np.arange(t_start,t_end, tau), PosList, SpeedList, AccselList )




