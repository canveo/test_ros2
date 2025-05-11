from PyQt5.QtWidgets import (QLabel, QVBoxLayout, QWidget, QMainWindow)

class StatsWindow(QMainWindow):
    def __init__(self, parent=None, controller=None):
        super(StatsWindow, self).__init__(parent)

        self.controller = controller
        self.setWindowTitle("Metrics")
        wid = QWidget(self)
        self.setCentralWidget(wid)

        self.layout = QVBoxLayout()
        m = self.controller.experiment_metrics

        def safe_label(text, key, unit=""):
            return QLabel(f"{text} -> {str(m.get(key, 'N/A'))}{unit}")

        self.layout.addWidget(safe_label("Percentage completed", 'percentage_completed', "%"))
        self.layout.addWidget(safe_label("Completed distance", 'completed_distance', " m"))
        self.layout.addWidget(safe_label("Average speed", 'average_speed', " m/s"))
        self.layout.addWidget(safe_label("Position deviation MAE", 'position_deviation_mae'))
        self.layout.addWidget(safe_label("Position deviation total error", 'position_deviation_total_err'))
        self.layout.addWidget(safe_label("Mean brain iterations real time", 'mean_brain_iterations_real_time', " s"))
        self.layout.addWidget(safe_label("Brain iterations frequency real time", 'brain_iterations_frequency_real_time', " it/s"))
        self.layout.addWidget(safe_label("Target brain iterations real time", 'target_brain_iterations_real_time', " it/s"))
        self.layout.addWidget(safe_label("Brain iterations frequency simulated time", 'brain_iterations_frequency_simulated_time', " it/s"))
        self.layout.addWidget(safe_label("Target brain iterations simulated time", 'target_brain_iterations_simulated_time', " it/s"))
        self.layout.addWidget(safe_label("Mean inference time", 'mean_inference_time', " s"))
        self.layout.addWidget(safe_label("Frame rate", 'frame_rate', " fps"))
        self.layout.addWidget(safe_label("Suddenness distance", 'suddenness_distance'))
        self.layout.addWidget(safe_label("GPU inference", 'gpu_inference'))
        self.layout.addWidget(safe_label("Mean brain iterations simulated time", 'mean_brain_iterations_simulated_time'))
        self.layout.addWidget(safe_label("Real time factor", 'real_time_factor'))
        self.layout.addWidget(safe_label("Real time update rate", 'real_time_update_rate'))
        self.layout.addWidget(safe_label("Experiment total simulated time", 'experiment_total_simulated_time', " s"))
        self.layout.addWidget(safe_label("Experiment total real time", 'experiment_total_real_time', " s"))

        if 'lap_seconds' in m:
            self.layout.addWidget(safe_label("Lap seconds", 'lap_seconds', " s"))
            self.layout.addWidget(safe_label("Circuit diameter", 'circuit_diameter', " m"))

        wid.setLayout(self.layout)

class CARLAStatsWindow(QMainWindow):
    def __init__(self, parent=None, controller=None):
        super(CARLAStatsWindow, self).__init__(parent)

        self.controller = controller
        self.setWindowTitle("Metrics")
        wid = QWidget(self)
        self.setCentralWidget(wid)

        self.layout = QVBoxLayout()
        m = self.controller.experiment_metrics

        def safe_label(text, key, unit=""):
            return QLabel(f"{text} -> {str(m.get(key, 'N/A'))}{unit}")

        keys = [
            ("Completed distance", 'completed_distance', " m"),
            ("Effective completed distance", 'effective_completed_distance', " m"),
            ("Average speed", 'average_speed', " km/h"),
            ("Experiment total real time", 'experiment_total_real_time', " s"),
            ("Experiment total simulated time", 'experiment_total_simulated_time', " s"),
            ("Collisions", 'collisions'),
            ("Lane invasions", 'lane_invasions'),
            ("Position deviation mean", 'position_deviation_mean', " m"),
            ("Position deviation total error", 'position_deviation_total_err'),
            ("Mean brain iterations real time", 'mean_brain_iterations_real_time', " s"),
            ("Brain iterations frequency real time", 'brain_iterations_frequency_real_time', " it/s"),
            ("Target brain iterations real time", 'target_brain_iterations_real_time', " it/s"),
            ("Mean brain iterations simulated time", 'mean_brain_iterations_simulated_time', " s"),
            ("Brain iterations frequency simulated time", 'brain_iterations_frequency_simulated_time', " it/s"),
            ("GPU mean inference time", 'gpu_mean_inference_time', " s"),
            ("GPU inference frequency", 'gpu_inference_frequency', " it/s"),
            ("GPU inference", 'gpu_inference'),
            ("Suddenness distance control commands", 'suddenness_distance_control_commands'),
            ("Suddenness distance throttle", 'suddenness_distance_throttle'),
            ("Suddenness distance steer", 'suddenness_distance_steer'),
            ("Suddenness distance brake command", 'suddenness_distance_brake_command'),
            ("Suddenness distance control command per km", 'suddenness_distance_control_command_per_km'),
            ("Suddenness distance throttle per km", 'suddenness_distance_throttle_per_km'),
            ("Suddenness distance steer per km", 'suddenness_distance_steer_per_km'),
            ("Suddenness distance brake command per km", 'suddenness_distance_brake_command_per_km'),
            ("Suddenness distance speed", 'suddenness_distance_speed'),
            ("Suddenness distance speed per km", 'suddenness_distance_speed_per_km')
        ]

        for label_text, key, *unit in keys:
            self.layout.addWidget(safe_label(label_text, key, unit[0] if unit else ""))

        if 'dangerous_distance_pct_km' in m:
            self.layout.addWidget(safe_label("Percentage of dangerous distance per km", 'dangerous_distance_pct_km'))
            self.layout.addWidget(safe_label("Percentage of close distance per km", 'close_distance_pct_km'))
            self.layout.addWidget(safe_label("Percentage of medium distance per km", 'medium_distance_pct_km'))
            self.layout.addWidget(safe_label("Percentage of great distance per km", 'great_distance_pct_km'))

        wid.setLayout(self.layout)
