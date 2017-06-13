Controller for Pheeno Navigation
================================

This document describes the controller design for the motors that drive Pheeno as well as the higher level controller that navigates the robot from one location to another. To create these controllers accurate models of the motors and robot are needed. The choice of models and fitting will be discussed in the :ref:`MotorModel` and :ref:`RobotModel` sections. The controller design sections will address which sensors are used to calculate the desired feedback but the details are discussed in the observer design document and thus, will not be talked about here.

The simplest possible models are used to design controllers for Pheeno. To understand this choice, refer to the passage from George Box's 1978 paper,

    Now it would be very remarkable if any system existing in the real world could be exactly represented by any simple model. However, cunningly chosen parsimonious models often do provide remarkably useful approximations. For example, the law :math:`PV = RT` relating pressure :math:`P`, volume :math:`V` and temperature :math:`T` of an "ideal" gas via a constant R is not exactly true for any real gas, but it frequently provides a useful approximation and furthermore its structure is informative since it springs from a physical view of the behavior of gas molecules.

    For such a model there is no need to ask the question "Is the model true?". If "truth" is to be the "whole truth" the answer must be "No". The only question of interest is "Is the model illuminating and useful?"

The better known section header of this passage is,

    All models are wrong but some are useful.

There is a need for the model to capture the properties of the system that are useful without overparamertizing or overelaborating the model. Using more intricate models than are needed makes the controller design more complicated and, more importantly, over fits the data used to parameterize the system which can cause this model to describe a certain data set more than the system it is meant to represent. With the use of feedback control, even simple models that do not describe the system completely can be controlled in desirable fashion.


.. _MotorModel:

###################
Modeling the Motors
###################

The first and arguably most important step towards controlling the system is designing a fast controller for the motors. To do this an accurate model of the motors must be created. Typically, this would be done by parameterizing the standard second order transfer function for a direct current (DC) motor,

.. math::

    P(s) = \frac{\dot{\Theta}(s)}{E_a(s)} = \frac{K_T}{(Js+b)(Ls+R)+K_BK_T}.


where :math:`K_T` is the gain relating the armature current to armature torque, :math:`K_B` is the gain relating the rotational velocity of the rotor to the back EMF in the armature circuit, :math:`J` is the inertial of the rotor, :math:`b` is the viscous friction acting on the rotor, :math:`L` is the inductance in the armature circuit, and :math:`R` is the resistance in the armature circuit.

However, for micro-metal gear motors, like the ones used on Pheeno, :math:`L << R`. This allows for the second order system to be approximated by a first order system by setting :math:`L=0` yielding,

.. math::

    P(s) \approx \frac{K_T}{R(Js+b)+K_BK_T} = \frac{K}{Ts + 1}


where,

.. math::

    K = \frac{K_T}{Rb+K_TK_B} \hspace{1cm} T = \frac{RJ}{Rb+K_TK_B}.


To fit these parameters, the motors will be black box modeled using MATLAB's system identification toolbox \cite{ljung2017system}. Two motors attached to Pheeno were given step voltages and sinusoidal voltages of frequencies between :math:`0.1` and :math:`50` Hz. The max sinusoidal frequency was chosen following the Nyquist sampling criterion which states a signal may only be recreated if it is sampled twice as fast as its highest frequency component. The rotor rotational velocities were measured with its encoders at a rate of :math:`100` Hz. The input and output time plots are shown in :ref:`Figure 1 <FigMotorInputOutput>`.

|


.. _FigMotorInputOutput:

.. figure:: ../assets/controls/MotorInputOutputTimePlot.jpg
   :scale: 25%
   :align: center
   :alt: Figure 1: Input and output signals for motor system identification.

   **Figure 1:** The voltage input (*top*) and rotor velocity output (*bottom*) signals for a DC motor.


|

From these plots, it is apparent there is a large magnitude drop off as the frequency of the input signal increases. This is expected in a natural system like the DC motor.

This data is given to the system identification toolbox to fit a first and second order continuous model of the motor. As expected, the first and second order transfer functions yield equal "goodness of fit" to the data (:math:`\sim 80\%`). This fit is a little low but acceptable as a linear model is unable to represent nonlinear friction effects on the motor rotor. The first and second order system both have a pole at :math:`\sim 36` rad/s with the second order system also having a pole at :math:`\sim 60,000` rad/s. From this fitting it is obvious the dynamics of the motor are dominated by the single pole. The location of the pole fit also makes sense as the magnitude of the sinusoidal response begins to decay around :math:`30` rad/s as is expected with roll off caused by a pole.

This leaves the first order approximation of the system.

.. math::

    P(s) = \frac{2.9876}{s+36.07}


The model compared to a set of validation data can be seen in :ref:`Figure 2 <FigMotorModelValidation>`.

|


.. _FigMotorModelValidation:

.. figure:: ../assets/controls/MotorOutputModel.jpg
    :scale: 35%
    :align: center
    :alt: Figure 2a: Motor Output Model

    *(a)*

.. figure:: ../assets/controls/MotorOutputModel2.jpg
    :scale: 35%
    :align: center
    :alt: Figure 2b: Motor Output Model Validation

    *(b)*

    **Figure 2:** The first order DC motor model (*red*) compared to validation data (*black*)


|


.. _MotorControl:

######################
Controlling the Motors
######################


.. _ContConsider:

************************
Continuous Consideration
************************

From this plant it is possible to design a controller so that the motor responds in a desired way. For this application it is desirable to make the motors follow step commands (and rejects step disturbances) as fast as possible with no overshoot.  From the internal model principle, a proprtional integral (PI) controller is enough to satisfy these requirements.

|


.. _FigFeedbackLoop:

.. figure:: ../assets/controls/FeedbackLoop.png
    :scale: 35%
    :align: center
    :alt: Figure 3: Block diagram of a standard feedback loop.

    **Figure 3:** A block diagram representation of a standard negative feedback loop.


|

This formulation leads to a standard feedback problem that must be solved. :ref:`Figure 3 <FigFeedbackLoop>` shows a block diagram of standard negative feedback loop with the symbols that will be used here. The plant, :math:`P(s)=\frac{2.9876}{s+36.07}`, was found previously, a PI controller, :math:`K=g\frac{s+a}{s}`, needs to be designed, and a pre-filter, :math:`W=\frac{a}{s+a}`, needs to be incorporated to reduce the overshoot caused by the zero of the controller. Here, the sensor dynamics, :math:`H`, are assumed to be ideal.

Typically, in a classroom setting, a pole placement method would be employed to match the nominal closed loop system to some desired closed loop system. In this case, a slightly different method will be employed. The plant here is a nominal representation of the system. This means there are higher order dynamics present in the system that are not modeled so the gain chosen cannot be very high and the dominant pole modeled here can be slightly or significantly off. First, :math:`a` will be chosen with these real world constraints in mind, then :math:`g` will be chosen such that there is no overshoot and the motors spin up as fast as possible.

First, notice the pre-filter is a stable low pass filter (presuming :math:`a > 0`). This means it will create a natural delay on the reference signal depending on how small :math:`a` is chosen. Thus :math:`a` should be chosen to be large such that this delay does not significantly slow down the motor's response to low frequency reference commands and does not restrict the magnitude of desired higher frequency signals. However, it should be chosen low enough that the integral term of the controller does not become too large and exacerbate the integral windup problem caused by motor saturation (which will be addressed later). Secondly, the proposed open loop system's (:math:`KP`) root locus will result in two different looking root locus depending on how :math:`a` is chosen. :ref:`Figure 4 <FigContRLocus>` shows an example of both root locus. Note choosing :math:`a` larger than the dominant pole of the modeled motor has two critical damping points. The left critical dampin point requires a much larger gain and thus will not be considered. The right critical point will shift left as :math:`a` is chosen closer to the pole of :math:`P` making the closed loop system's response faster. To make the system as fast as possible, choose :math:`a = 36.07` so it cancels the pole of the motor. Typically, this is not a good idea because of modeling errors that cause this cancellation to not be true in reality. However, because it is a very stable pole, this choice does not have such severe consequences as model error will result in one of the two root locus represented in :ref:`Figure 4 <FigContRLocus>`.

|


.. _FigContRLocus:

.. figure:: ../assets/controls/Rlocus1.jpg
    :scale: 35%
    :align: center
    :alt: Figure 4a: Root locus for the continuous motor control design.

    *(a)*

.. figure:: ../assets/controls/Rlocus2.jpg
    :scale: 35%
    :align: center
    :alt: Figure 4b: Root locus for the continuous motor control design.

    *(b)*

    **Figure 4:** The root locus of Pheeno's motors where the zero of the controller is chosen larger (*top*) and smaller (*bottom*) than the modeled pole of the motor.


|

The gain, :math:`g`, is chosen such that the rise time of the system is math:`<0.5` s. :ref:`Figure 5 <FigMotorSimStep>` shows a simulated step response of this controller and modeled plant which behaves as expected.

|


.. _FigMotorSimStep:

.. figure:: ../assets/controls/MotorModelStepResponse.jpg
    :scale: 35%
    :align: center
    :alt: Figure 5: Simulated step response of the continuous motor control loop.

    **Figure 5:** The simulated step response of the designed continuous motor control loop.


|


.. _DealIntegralWindup:

****************************
Dealing with Integral Windup
****************************

Integral windup is a problem that occurs in controllers containing integral terms when a large change in the reference command occurs. For example, if Pheeno is suddenly commanded to go from rest to full speed there will be a large error initially that will get smaller as Pheeno begins to reach its top speed. However, during this time the integral term of the controller will be compounding this error and cause an overshoot until enough error has occurred in the opposite direction to offset it. This gets worse when saturation of actuators are considered. In this case, a naive controller can require an output larger than what can be produced by the actuator. This causes integration error to continue to compile without knowledge that the error it is trying to rectify is beyond the capabilities of the system.

There are several ways to address this issue. In Pheeno, this issue is addressed by adding a secondary feedback loop that limits the integral term when motor saturation has occurred. :ref:`Figure 6 <FigWindupPI>` shows this feedback loop in block diagram form. This feedback loop only kicks in if the controller's desired output is higher or lower than the actuator can output. When saturation occurs, the feedback loop keeps the integral term from compounding when the error cannot be reduced. Typically the tracking gain, :math:`K_t`, is chosen to be equal to the integral gain, :math:`K_i`, but higher values can cause better performance \cite{bohn1994simulink}.

|


.. _FigWindupPI:

.. figure:: ../assets/controls/WindupPI.png
    :scale: 35%
    :align: center
    :alt: Figure 6: Comparison of model to motor output for several reference commands.

    **Figure 6:** Comparison of model to motor output for several reference commands.


|


.. _DiscreteTimeAdapt:

************************
Discrete Time Adaptation
************************

It would be naive to just throw this controller onto the robot and assume the motors will respond as they were designed to. If the controller were designed in the continuous domain without accounting for the delays created by the sampling of the microcontroller, it is very likely the control would be unstable at worse or not exhibit the designed properties at best. The Teensy microcontroller can easily perform control loops at :math:`100` Hz. Thus a sampling time of :math:`0.01` is chosen to design the motor control around.

First, the plant should be transformed from the continuous domain to the discrete domain. There are many options to transform a continuous plant represented in the s-domain to the discrete time z-domain. For the plant, a zero-order hold (ZOH) conversion is chosen as that best represents the type of hold circuit that will be used in sampling the motor. The controller designed in the continuous case is transformed using the bilinear transformation to better approximate the continuous behavior of the controller in the discrete space.

This control case is ideal as the sampling time is very fast compared to the desired rise time. Thus, the continuous system is very close to the discrete system. :ref:`Figure 7 <FigDiscretevsContinuousMotorModel>` shows the step response of the discrete designed system with the continuous designed system. However, in general this will not be the case and the continuous system would need to be augmented with an additional gain to get the desired response characteristics or redesigned entirely.

To validate this control, several known commands are given to two different motors on two different robots. The model's prediction is compared to the motor outputs in :ref:`Figure 8 <FigMotorValidation>`. For large jumps in the reference command, like the last two step commands, the integral windup overshoot is apparent but not overwhelming. This could be remedied with a less aggressive controller or putting a more restrictive low pass pre-filter on the reference commands. This would slow down the response considerably which is undesirable.

|


.. _FigDiscretevsContinuousMotorModel:

.. figure:: ../assets/controls/MotorCDModelStepResponse.jpg
    :scale: 35%
    :align: center
    :alt: Figure 7: Comparison of simulated step response of the continuous and discrete motor control loop.

    **Figure 7:** Comparison of simulated step response of the designed continuous and discrete motor control loop. The *blue* line shows the discrete time step response and the *red* line shows the continuous step response.


|


.. _FigMotorValidation:

.. figure:: ../assets/controls/MotorValidationTimePlot.jpg
    :scale: 35%
    :align: center
    :alt: Figure 8: Comparison of model to motor output for several reference commands.

    **Figure 8:** Comparison of model to motor output for several reference commands.


|


.. _RobotModel:

##################
Modeling the Robot
##################

Pheeno is by default a differential drive robot. This means each wheel can be controlled independently to produce desired motion. However, this makes the robot a coupled system resulting in a multi input multi output system which can be tricky to control properly. To simplify this, a decoupled kinematic model is used to represent Pheeno's motion and control its position and orientation in a global reference frame. This can be done since Pheeno is so light and its motion is dominated by the motor torques. An extremely in depth analysis about when this assumption can be used is done by \citet{anvari2013non} in his master's thesis.

|


.. _FigPheenoControlReferenceFrame:

.. figure:: ../assets/controls/PheenoReferenceFrame.png
    :scale: 35%
    :align: center
    :alt: Figure 9: Representation of Pheeno in a Cartesian coordinate frame.

    **Figure 9:** Representation of Pheeno in a Cartesian coordinate frame.


|

Consider Pheeno in an inertial reference frame :math:`\{X_o, Y_o\}` as shown in :ref:`Figure 9 <FigPheenoControlReferenceFrame>`. Pheeno's basic motion model is, what is commonly referred to as, the unicycle model.

.. math::
    :label: UnicycleWV

    \begin{bmatrix}
    \dot{x} \\ \dot{y} \\ \dot{\theta}
    \end{bmatrix}
    =
    \begin{bmatrix}
    \cos{\theta} & 0 \\
    \sin{\theta} & 0 \\
    0 & 1
    \end{bmatrix}
    \begin{bmatrix}
    v \\ w
    \end{bmatrix}


This model transforms the robot's linear velocity, :math:`v`, and rotational velocity, :math:`w`, in Pheeno's reference frame to velocity states in the inertial frame. However, the robot's linear and rotational velocity cannot be controlled directly so another transformation is needed linking the rotational velocity of the wheels, :math:`v_R` and :math:`v_L`, to :math:`v` and :math:`w`. This relation is derived more thoroughly in \citet{malu2014kinematics}.

.. math::
    :label: UnicycleConversion

    \begin{bmatrix}
    v_R \\ v_L
    \end{bmatrix}
    =
    \begin{bmatrix}
    \frac{1}{r} & \frac{L}{2r} \\
    \frac{1}{r} & \frac{-L}{2r}
    \end{bmatrix}
    \begin{bmatrix}
    v \\ w
    \end{bmatrix}


Here, :math:`r` is the wheel radius and :math:`L` is the axle length of the differential drive robot. Combining \autoref{eq:UnicycleWV} and \autoref{eq:UnicycleConversion}, yields the final relation between the wheel speeds of the robot and the velocity states in the inertial reference frame.

.. math::
    :label: UnicycleRL

    \begin{bmatrix}
    \dot{x} \\ \dot{y} \\ \dot{\theta}
    \end{bmatrix}
    =
    \begin{bmatrix}
    \frac{r}{2}\cos{\theta} & \frac{r}{2}\cos{\theta} \\
    \frac{r}{2}\sin{\theta} & \frac{r}{2}\sin{\theta} \\
    \frac{r}{L} & \frac{-r}{L}
    \end{bmatrix}
    \begin{bmatrix}
    v_R \\ v_L
    \end{bmatrix}


However, it is much more intuitive to use the unicycle model (\autoref{eq:UnicycleWV}) thus control will be done to create reference linear velocities, :math:`v`, and rotational velocities, :math:`w`, for the robot to follow. These will then be transformed to motor velocity commands using \autoref{eq:UnicycleConversion}.

In discrete time, this unicycle model takes the form,

.. math::
    :label: DUnicycleWV

    \begin{bmatrix}
    x \\ y \\ \theta
    \end{bmatrix}
    _{k+1}
    =
    \begin{bmatrix}
    1 & 0 &0\\
    0 & 1 & 0 \\
    0 & 0 & 1
    \end{bmatrix}
    \begin{bmatrix}
    x \\ y \\ \theta
    \end{bmatrix}
    _k
    +
    \begin{bmatrix}
    \Delta t \cos({\theta_k + \frac{\Delta\theta_k}{2}}) & 0 \\
    \Delta t \sin({\theta_k + \frac{\Delta\theta_k}{2}}) & 0 \\
    0 & \Delta t
    \end{bmatrix}
    \begin{bmatrix}
    v \\ w
    \end{bmatrix}


This unicycle model has slight changes to the orientation model that can be found in a paper by \citet{kiriy2002three}. Using this model over the usual one showed vast improvements in dead reckoning navigation for Pheeno.


.. _RobotControl:

##############################
Controlling the Robot's Motion
##############################

The approach to using the unicycle model to navigate Pheeno from an initial position to a goal position described here is using a layered architecture. This means using a high level planner to design way points for the robot to pass through, which are then translated to linear and rotational velocities of the robot, which are finally put through the fast PI controller of the motors. This section focuses on the middle component which decides the set points for the linear and rotational velocities.

Assume the high level planner has given an initial desired position :math:`\vec{u} = [u_x \hspace{2mm} u_y \hspace{2mm} u_\theta]^T`. From a Lyapunov stability analysis in \cite{malu2014kinematics} the controllers which produce stable global position tracking are,

.. math::

    v = K_p \rho \cos{\alpha}


.. math::

    w = K_p \sin{\alpha}\cos{\alpha} + K_\alpha \alpha


where :math:`K_p > 0` is a gain associated with radial distance error from the goal location, :math:`\rho = \sqrt{(u_x - x)^2 + (u_y - y)^2}`, and :math:`K_\alpha > 0` is a gain associated with the robot's heading error from the goal orientation, :math:`\alpha = atan2(\frac{u_y - y}{u_x - x})`. It should be noted the heading error, :math:`\alpha`, is bounded :math:`[-\pi, \pi]` which limits how large :math:`w` can get. However, the radial distance error, :math:`\rho`, is unbounded. Thus, it is typical in application to either know the bounds of :math:`\rho` when designing :math:`K_p` as a constant or choosing :math:`K_p` to be of the form,

.. math::

    K_p =\frac{v_0(1-e^{-a\rho^2})}{\rho}


which limits the maximum linear velocity of the robot to a designed :math:`v_0`. When designing for any application, the gains should be chosen carefully to avoid wheel slip caused by high accelerations. The controllers should also operate slower than the rise time of the motor controller (:math:`\sim 0.1` s) so the motors have a chance to produce the desired linear and rotational velocities demanded by the higher order controller.


:raw-tex:`\cite{box1976science}`

.. raw:: latex

 \bibliographystyle{plain}
 \bibliography{ControllerDesign.bib}
