# PID Control System

## 📌 Project Overview
This project implements a **Proportional-Integral-Derivative (PID) Controller** for dynamic systems, allowing precise control over a system’s response. The PID controller is used to minimize the error between the desired setpoint and the actual process output.

The project includes:
- **Mathematical modeling** of the system using transfer functions.
- **Cohen-Coon method** for initial PID parameter estimation.
- **Optimization techniques** to refine PID parameters for robustness.
- **Simulation of system response** using Python and Control Systems Toolbox.

## 🛠 Features
- **Automatic PID Tuning**: Uses the **Cohen-Coon method** to determine initial PID parameters.
- **Optimization for Performance**: Refines parameters using error minimization techniques.
- **Visualization of Step Response**: Plots system performance under PID control.
- **Adjustable Robustness Levels**: Supports different robustness settings based on application needs.

## 📁 Project Structure
```
│── PID_Kontrol.pdf          # Project documentation
│── pid_controller.py        # Main PID implementation script
│── simulation_results.png   # Sample simulation results
│── README.md                # Project description and usage guide
```

## 🔧 Dependencies
Ensure you have the following Python libraries installed:
```bash
pip install numpy scipy control matplotlib
```

## 🚀 How to Use
### 1️⃣ Define System Transfer Function
Modify the numerator and denominator coefficients in `calculate_pid()`:
```python
num = [1]  # Numerator coefficients
den = [1, 3, 3, 1]  # Denominator coefficients
feedback_num = [1]
feedback_den = [1]
```

### 2️⃣ Run the PID Controller
Execute the script:
```bash
python pid_controller.py
```
The system response will be plotted, showing how the PID controller optimizes performance.

### 3️⃣ Adjust Robustness and Response Time
Modify the following parameters to tune the controller for different stability levels:
```python
robustness = 'High Robustness'  # Options: Low, Medium, High, Very High
response_time = 1.0  # Adjust for faster or slower response
```

## 📊 Results
The optimized PID parameters will be displayed in the terminal:
```
Refined PID Parameters:
Kp: 2.3
Ki: 1.5
Kd: 0.8
```
The step response graph will also be generated to visualize the control performance.

## 📌 Future Improvements
- Implement **adaptive PID tuning** for real-time parameter adjustments.
- Extend the project for **nonlinear systems** and **digital controllers**.
- Enhance robustness against **external disturbances**.

## 📜 License
This project is released under the **MIT License**.

## 📬 Contact
For questions or contributions, feel free to open an issue or reach out!
