import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import tkinter as tk
from tkinter import messagebox

def calculate_pid(num, den, feedback_num, feedback_den, robustness, response_time):
    system = ctrl.TransferFunction(num, den)
    feedback_system = ctrl.TransferFunction(feedback_num, feedback_den)

    # Calculate initial PID parameters using Cohen-Coon and Ziegler-Nichols methods
    cohen_coon_params = cohen_coon_tuning(system, feedback_system)
    ziegler_nichols_params = ziegler_nichols_tuning(system, feedback_system)
    optimized_params = refine_pid_parameters(system, feedback_system, 1.0, 1.0, 1.0, np.linspace(0, 200, 2000), robustness, response_time)
    
    if cohen_coon_params is None and ziegler_nichols_params is None and optimized_params is None:
        raise ValueError("Failed to calculate PID parameters using any method.")
    
    methods = {
        "Cohen-Coon": cohen_coon_params,
        "Ziegler-Nichols": ziegler_nichols_params,
        "Optimization": optimized_params
    }

    best_method = min(methods, key=lambda k: evaluate_method(system, feedback_system, methods[k], robustness, response_time))
    best_params = methods[best_method]
    
    # Apply best PID parameters
    controller = ctrl.TransferFunction([best_params[2], best_params[0], best_params[1]], [1, 0])
    closed_loop = ctrl.feedback(controller * system, feedback_system)
    t, y = ctrl.step_response(closed_loop, T=np.linspace(0, 200, 2000))

    # Plot the response
    plt.plot(t, y, label="PID Response")
    plt.axhline(y=1, color='r', linestyle='--', label="Setpoint")
    plt.title(f'PID Controlled System Response ({best_method} Method)')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.legend()
    plt.grid(True)
    plt.show()

    result_text = (
        f"Best PID Parameters ({best_method} Method):\n"
        f"Kp: {best_params[0]}\n"
        f"Ki: {best_params[1]}\n"
        f"Kd: {best_params[2]}"
    )
    print(result_text)
    return best_params

def cohen_coon_tuning(system, feedback_system):
    t = np.linspace(0, 200, 2000)
    t, y = ctrl.step_response(system, T=t)

    # Find time constant (tau) and dead time (L)
    L = np.argmax(np.diff(y)) * (t[1] - t[0])
    tau = t[np.where(y >= 0.632 * y[-1])[0][0]] - L
    K = y[-1]

    if L <= 0 or tau <= 0:
        return None

    # Cohen-Coon tuning formulas
    Kp = (1.35 / K) * (tau / L) ** 0.27
    Ki = Kp / (2.5 * L)
    Kd = 0.37 * Kp * L

    return Kp, Ki, Kd

def ziegler_nichols_tuning(system, feedback_system):
    t = np.linspace(0, 200, 2000)
    Kp = 1
    controller = ctrl.TransferFunction([0, Kp], [1])
    closed_loop = ctrl.feedback(controller * system, feedback_system)
    _, y = ctrl.step_response(closed_loop, T=t)

    # Detect oscillations
    oscillation_indices = np.where(np.diff(np.sign(np.diff(y))) != 0)[0]
    if len(oscillation_indices) < 2:
        return None

    Pu = (t[oscillation_indices[1]] - t[oscillation_indices[0]]) * 2
    Ku = 4 / (np.max(y) - np.min(y))

    if Ku <= 0 or Pu <= 0:
        return None

    # Ziegler-Nichols tuning formulas
    Kp = 0.6 * Ku
    Ki = 2 * Kp / Pu
    Kd = Kp * Pu / 8

    return Kp, Ki, Kd

def refine_pid_parameters(system, feedback_system, Kp, Ki, Kd, t, robustness, response_time, max_iter=1000):
    best_error = float('inf')
    best_params = (Kp, Ki, Kd)
    
    def objective(params):
        Kp, Ki, Kd = params
        controller = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])
        closed_loop = ctrl.feedback(controller * system, feedback_system)
        _, y = ctrl.step_response(closed_loop, T=t)
        
        # Penalize squared error and add penalties for robustness and response time
        error = np.sum((y - 1) ** 2)  # Minimize squared error
        
        max_deviation = np.max(np.abs(y - 1))
        sum_of_diff = np.sum(np.abs(np.diff(y)))
        max_of_diff = np.max(np.abs(np.diff(y)))
        
        # Adjusted penalty calculation
        robustness_penalty = {
            'Low': 50,
            'Medium': 100,
            'High': 500,
            'Very High': 950
        }.get(robustness, 50)

        error += robustness_penalty * (max_deviation + sum_of_diff + 0.5 * max_of_diff)
        
        # Add penalty for response time
        response_penalty = np.sum(np.abs(np.diff(y)) > 1e-2)
        error += response_penalty * response_time

        # Check for NaNs or Infs
        if np.isnan(error) or np.isinf(error):
            return float('inf')

        print(f"Params: Kp={Kp}, Ki={Ki}, Kd={Kd} => Error: {error}")  # Added detailed logging

        return error

    print(f"Initial guess: Kp={Kp}, Ki={Ki}, Kd={Kd}")
    result = minimize(objective, [Kp, Ki, Kd], bounds=[(0, None), (0, None), (0, None)], method='Powell', options={'maxiter': max_iter})
    
    if result.success:
        best_params = result.x
        print(f"Optimization succeeded: {result}")
    else:
        raise ValueError(f"Optimization failed: {result.message}")
        
    return best_params

def evaluate_method(system, feedback_system, params, robustness, response_time):
    if params is None:
        return float('inf')
    Kp, Ki, Kd = params
    t = np.linspace(0, 200, 2000)
    controller = ctrl.TransferFunction([Kd, Kp, Ki], [1, 0])
    closed_loop = ctrl.feedback(controller * system, feedback_system)
    _, y = ctrl.step_response(closed_loop, T=t)
    
    # Penalize squared error and add penalties for robustness and response time
    error = np.sum((y - 1) ** 2)  # Minimize squared error
    
    max_deviation = np.max(np.abs(y - 1))
    sum_of_diff = np.sum(np.abs(np.diff(y)))
    max_of_diff = np.max(np.abs(np.diff(y)))
    
    # Adjusted penalty calculation
    robustness_penalty = {
        'Low': 50,
        'Medium': 80,
        'High': 300,
        'Very High': 2290
    }.get(robustness, 50)

    error += robustness_penalty * (max_deviation + sum_of_diff + 0.5 * max_of_diff)
    
    # Add penalty for response time
    response_penalty = np.sum(np.abs(np.diff(y)) > 1e-2)
    error += response_penalty * response_time

    return error

def run_gui():
    root = tk.Tk()
    root.title("PID Parameter Tuning")

    def submit():
        try:
            num = list(map(float, num_entry.get().split(',')))
            den = list(map(float, den_entry.get().split(',')))
            feedback_num = list(map(float, feedback_num_entry.get().split(',')))
            feedback_den = list(map(float, feedback_den_entry.get().split(',')))
            
            if len(num) >= len(den):
                raise ValueError("Denominator must have a higher order than the numerator.")

            robustness = robustness_var.get()
            response_time = float(response_time_entry.get())
            Kp, Ki, Kd = calculate_pid(num, den, feedback_num, feedback_den, robustness, response_time)
            result_text = f"Calculated PID parameters:\nKp: {Kp}\nKi: {Ki}\nKd: {Kd}"
            messagebox.showinfo("Result", result_text)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    tk.Label(root, text="Numerator Coefficients (comma separated):").grid(row=0, column=0)
    num_entry = tk.Entry(root)
    num_entry.grid(row=0, column=1)

    tk.Label(root, text="Denominator Coefficients (comma separated):").grid(row=1, column=0)
    den_entry = tk.Entry(root)
    den_entry.grid(row=1, column=1)

    tk.Label(root, text="Feedback Numerator Coefficients (comma separated):").grid(row=2, column=0)
    feedback_num_entry = tk.Entry(root)
    feedback_num_entry.grid(row=2, column=1)

    tk.Label(root, text="Feedback Denominator Coefficients (comma separated):").grid(row=3, column=0)
    feedback_den_entry = tk.Entry(root)
    feedback_den_entry.grid(row=3, column=1)

    tk.Label(root, text="Robustness (Low, Medium, High, Very High):").grid(row=4, column=0)
    robustness_var = tk.StringVar()
    robustness_menu = tk.OptionMenu(root, robustness_var, "Low", "Medium", "High", "Very High")
    robustness_menu.grid(row=4, column=1)

    tk.Label(root, text="Response Time:").grid(row=5, column=0)
    response_time_entry = tk.Entry(root)
    response_time_entry.grid(row=5, column=1)

    tk.Button(root, text="Submit", command=submit).grid(row=6, columnspan=2)

    root.mainloop()

if __name__ == "__main__":
    run_gui()