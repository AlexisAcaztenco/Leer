import control as ct
import matplotlib.pyplot as plt
import numpy as np
# System parameters
m = 4 # mass of aircraft
J = 0.0475 # inertia around pitch axis/inercia alrededor del eje de cabeceo
r = 0.25 # distance to center of force/distancia al centro de fuerza
g = 9.8 # gravitational constant/constante gravitacional
c = 0.05 # damping factor (estimated)/factor de amortiguamiento (estimado)
# Transfer functions for dynamics/ Funciones de transferencia para dinámica
Pi = ct.tf([r], [J, 0, 0]) # inner loop (roll)/bucle interior (rollo)
Po = ct.tf([1], [m, c, 0]) # outer loop (posn)/bucle exterior (posn)
# Inner loop control design
#
# Controller for the pitch dynamics: the goal is
# to have a fast response so that we can use this
# as a simplified process for the lateral dynamics
# Design a simple lead controller for the system
k_i, a_i, b_i = 200, 2, 50
Ci = k_i * ct.tf([1, a_i], [1, b_i])
Li = Pi * Ci
# Loop transfer function Bode plot, with margins
plt.figure(); ct.bode_plot(Li, margins=True)
plt.savefig('pvtol-inner-ltf.pdf')
# Make sure inner loop specification is met
plt.figure(); ct.gangof4_plot(Pi, Ci)
plt.savefig('pvtol-gangof4.pdf')

# Design lateral control system (lead compensator)
a_o, b_o, k_o = 0.3, 10, 2
Co = -k_o * ct.tf([1, a_o], [1, b_o])
Lo = -m * g * Po * Co
# Compute real outer-loop loop transfer function
L = Co * Pi * Po


# Compute stability margins
gm, pm, wgc, wpc = ct.margin(L)
# Check to make sure that the specification is met
plt.figure(); ct.gangof4_plot(-m * g * Po, Co)
# Nyquist plot for complete design
plt.figure(); ct.nyquist_plot(L)
plt.savefig('pvtol-nyquist.pdf')


# problema
# Step response
#t, y = ct.step_response(t, np.linspace(0, 20))
#plt.figure(); plt.plot(t, y)
#plt.savefig('pvtol-step.pdf')

plt.show()

