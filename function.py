import numpy as np
import matplotlib.pyplot as plt
import random

class parametrs:

    def __init__(self, start_position: float, start_speed: float, start_accsel: float, L: float, k_b: float, I: float, Tcmd: float ):

        self.g = 9.81

        self.position = start_position
        self.speed = start_speed
        self.accsel = start_accsel
        self.L = L
        self.k_b = k_b
        self.I = I
        self.Tcmd = Tcmd

    ## определим функции над данными величинами

    ##попытка учесть переворот квадрокоптера
    ##в таком случае винты должны начать вращаться в обратную сторону. Насколько это физически возможно вопрос открытый
    ##попытку учесть разворот считаю неудачной
    def angle_check(self, CurrentAnglePosition):

        print(CurrentAnglePosition)

        if (CurrentAnglePosition >= 0 and CurrentAnglePosition <= 90) or (CurrentAnglePosition > 270 and CurrentAnglePosition <= 360):

            sign_ = 0

        elif (CurrentAnglePosition > 90 and CurrentAnglePosition <= 270):

            sign_ = 1

        return 0.0

    def caclulation_omega(self, CurrentAngleAccsel):

        return np.asarray([ CurrentAngleAccsel + self.Tcmd, -CurrentAngleAccsel + self.Tcmd ])

    def calculation_acc(self, omega, sign_):

        Omega = self.caclulation_omega(omega)

        self.accsel = self.k_b * (Omega[0]**2 - Omega[1]**2) * (-1)**sign_ * self.L / self.I

    def calculation_speed(self, omega, sign_, tau):

        self.calculation_acc(omega, sign_)

        self.speed = self.speed + self.accsel * tau

    def calculation_position(self, omega, sign_, tau):

        self.calculation_speed(omega, sign_, tau)

        self.position = self.position + self.speed * tau

class control_function:

    def __init__(self, CorrectPosition: float, CorrectSpeed: float, K_p, K_i, K_d, max = 10000. ):

        self.CorrectPosition = CorrectPosition
        self.CorrectSpeed = CorrectSpeed
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d
        self.max = max

        self.IntegralError_speed = 0.0
        self.error_old_speed = 0.0

        self.IntegralError_acc = 0.0
        self.error_old_acc = 0.0

    #поставим ограничение на возможный угол поворота, в рамках которого еще действует физика
    def check_correct_position(self):

        if self.CorrectPosition > 90:

            self.CorrectPosition = 90

    def pid_regulation_speed(self, CurrentAnglePosition, tau):

        self.error = self.CorrectPosition - CurrentAnglePosition

        self.IntegralError_speed = self.IntegralError_speed + self.error * tau

        CurrentOmega = self.K_p[0] + self.error + self.K_i[0] * self.IntegralError_speed + self.K_d[0] * ( self.error - self.error_old_speed ) / tau

        self.error_old_speed = self.error

        if CurrentOmega > self.max:

            CurrentOmega = self.max

        elif CurrentOmega < -self.max:

            CurrentOmega = -self.max

        return CurrentOmega

    def pid_regulation_accselection(self, CurrentAngleSpeed, tau):

        self.error = CurrentAngleSpeed - self.CorrectSpeed

        self.IntegralError_acc = self.IntegralError_acc + self.error * tau

        CurrentAccsel = self.K_p[1] + self.error + self.K_i[1] * self.IntegralError_acc + self.K_d[1] * ( self.error - self.error_old_acc ) / tau

        if CurrentAccsel > self.max:

            CurrentAccsel = self.max

        elif CurrentAccsel < -self.max:

            CurrentAccsel = -self.max

        return CurrentAccsel

def quad_sim(param, control, tau, T):

    PosList = np.zeros(np.int64(T / tau))

    SpeedList = np.zeros(np.int64(T / tau))

    AccselList = np.zeros(np.int64(T / tau))

    control.check_correct_position()

    for i in np.arange(0, np.int64(T / tau) + 100, 1):

        if tau * i >= T:

            break

        else:

            AccselList[i] = param.accsel

            SpeedList[i] = param.speed

            PosList[i] = param.position

            omega = control.pid_regulation_speed(PosList[i], tau)

            accsel = control.pid_regulation_accselection(omega, tau)

            sign_ = param.angle_check(PosList[i])

            param.calculation_position(accsel, sign_, tau)

    return PosList, SpeedList, AccselList

def plot_graph(time, PosList, SpeedList, AccselList):

    fig = plt.figure(figsize=(10, 6))
    grid = fig.add_gridspec(3, 5)
    ax_position = fig.add_subplot(grid[0, :-1])
    ax_speed = fig.add_subplot(grid[1, :-1])
    ax_accsel = fig.add_subplot(grid[2, :-1])

    ax_position.plot(time, PosList, 'r-', label = 'position')
    ax_position.set_xlabel('time, sec')
    ax_position.set_ylabel('position')
    ax_position.grid()
    ax_position.legend()

    ax_speed.plot(time, SpeedList, 'b-', label ='speed')
    ax_speed.set_xlabel('time, sec')
    ax_speed.set_ylabel('speed')
    ax_speed.grid()
    ax_speed.legend()

    ax_accsel.plot(time, AccselList, 'g-', label = 'acceleration')
    ax_accsel.set_xlabel('time, sec')
    ax_accsel.set_ylabel('acceleration')
    ax_accsel.grid()
    ax_accsel.legend()

    plt.show()












