class PIDController:

    def __init__(self,beta,limits,gains,Ts):

        # Set all class values
        self.sat_limits = limits # lower limit (limit[0]) and upper limit (limit[1])
        # for saturation values
        self.beta = beta # dirty derivative value
        self.kP = gains[0]
        self.kD = gains[1]
        self.kI = gains[2]
        self.Ts = Ts

        # Set previous values to 0 initially
        self.y_old = 0
        self.error_old = 0
        self.uI_old = 0
        self.uD_old = 0

    def u(self, y_ref, y, u_eq):

        # Calculate error
        error = y_ref - y

        # Calculate uX values
        uP = error
        uI = self.uI_old + self.Ts/2.*(error + self.error_old)
        uD = self.beta*self.uD_old + (1-self.beta)/self.Ts*(y - self.y_old)

        # Find u_tilde
        u_unsat = self.kP*uP + self.kI*uI + self.kD*uD + u_eq

        # Saturate u
        u = self.saturate(u_unsat)

        # Integrator wind-up
        uI = (u - u_unsat)/self.kP + uI

        # update old values
        self.y_old = y
        self.error_old = error
        self.uI_old = uI
        self.uD_old = uD

        # return u
        return u

    def saturate(self, value):

        # test against limits
        if value < self.limits[0]:
            value = self.limits[0]
        elif value > self.limits[1]:
            value = self.limits[1]

        return value

    # end of class definition PIDController
