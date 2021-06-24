# task_turtle_sim

MPC with turtlesim based on unicycle model is in file <a href="https://github.com/suneet1212/task_turtle_sim/blob/main/src/turtle_task.cpp"> turtle_task.cpp </a>
<br>
MPC with turtlesim based on bicycle model is in file <a href="https://github.com/suneet1212/task_turtle_sim/blob/main/src/task.cpp"> task.cpp </a>
<br>


### Equations Used for bicycle model

x_next = x_current + velocity_current * cos(steering_angle + &theta;_current) * &Delta;T

y_next = y_current + velocity_current * sin(steering_angle + &theta;_current) * &Delta;T

&theta;_next = &theta;_current + velocity_current * sin(steering_angle) * &Delta;T / L



L = biycle model length
<br>
&Delta;T is the time period for mpc
