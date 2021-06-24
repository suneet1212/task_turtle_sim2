# task_turtle_sim

### Equations Used for bicycle model

x_next = x_current + velocity_current * cos(steering_angle + &theta;_current) * &Delta;T

y_next = y_current + velocity_current * sin(steering_angle + &theta;_current) * &Delta;T

&theta;_next = &theta;_current + velocity_current * sin(steering_angle) * &Delta;T / L
