import os
import yaml
from ament_index_python.packages import get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QPushButton
from qt_gui.plugin import Plugin
from rosidl_runtime_py.utilities import get_message

class TopicFrequencyMonitor(Plugin):
    def __init__(self, context):
        super(TopicFrequencyMonitor, self).__init__(context)
        self.setObjectName('TopicFrequencyMonitor')
        self.node = context.node

        # Find config file using ament_index
        config_file = os.path.join(
            get_package_share_directory('eufs_rqt'),
            'config',
            'topics_frequency_config.yaml'
        )

        self._widget = TopicFrequencyMonitorWidget(self.node, config_file)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (
                ' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

class TopicFrequencyMonitorWidget(QWidget):
    def __init__(self, node, config_path):
        super().__init__()
        self.node = node
        self.config_path = config_path
        
        # Set object name and window title like the working examples
        self.setObjectName('TopicFrequencyMonitorWidget')
        self.setWindowTitle('Topic Frequency Monitor')
        
        # Set up layout
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        
        # Create table
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(['Topic', 'Frequency (Hz)', 'Type'])
        self.table.setMinimumSize(400, 200)
        self.layout.addWidget(self.table)
        
        # Create reload button
        self.reload_button = QPushButton("Reload Topics")
        self.reload_button.clicked.connect(self.reload_topics)
        self.layout.addWidget(self.reload_button)
        
        # Initialize data structures
        self.subscribers = {}
        self.msg_times = {}
        self.types = {}
        self.topics = []
        
        # Load topics and start monitoring
        self.reload_topics()
        
        # Set up timer for updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_table)
        self.timer.start(1000)  # 1 Hz

    def reload_topics(self):
        # Unsubscribe old
        for sub in self.subscribers.values():
            self.node.destroy_subscription(sub)
        self.subscribers.clear()
        self.msg_times.clear()
        self.types.clear()
        
        # Load config
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.topics = config.get('topics', [])
            self.node.get_logger().info(f"Loaded {len(self.topics)} topics from config")
        except Exception as e:
            self.node.get_logger().error(f"Could not load config: {e}")
            self.topics = []
        
        # Set up monitoring for each topic
        for topic in self.topics:
            self.msg_times[topic] = []
            self.types[topic] = ''
            topic_type = self.get_topic_type(topic)
            if topic_type:
                try:
                    msg_class = get_message(topic_type)
                    self.types[topic] = topic_type
                    self.subscribers[topic] = self.node.create_subscription(
                        msg_class, topic, lambda msg, t=topic: self.callback(msg, t), 10)
                    self.node.get_logger().debug(f"Subscribed to {topic} ({topic_type})")
                except Exception as e:
                    self.node.get_logger().warn(f"Failed to subscribe to {topic}: {e}")
                    self.types[topic] = 'unknown'
            else:
                self.node.get_logger().warn(f"Topic {topic} not found or has no type")
                self.types[topic] = 'unknown'
        
        self.update_table()

    def get_topic_type(self, topic):
        for name, types in self.node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        return None

    def callback(self, msg, topic):
        now = self.node.get_clock().now().nanoseconds / 1e9
        self.msg_times[topic].append(now)

    def update_table(self):
        if not self.topics:
            self.table.setRowCount(1)
            self.table.setItem(0, 0, QTableWidgetItem("No topics configured"))
            self.table.setItem(0, 1, QTableWidgetItem(""))
            self.table.setItem(0, 2, QTableWidgetItem(""))
            return
            
        self.table.setRowCount(len(self.topics))
        now = self.node.get_clock().now().nanoseconds / 1e9
        for i, topic in enumerate(self.topics):
            times = [t for t in self.msg_times.get(topic, []) if now - t < 5.0]
            self.msg_times[topic] = times
            freq = len(times) / 5.0 if times else 0.0
            self.table.setItem(i, 0, QTableWidgetItem(topic))
            self.table.setItem(i, 1, QTableWidgetItem(f"{freq:.2f}"))
            self.table.setItem(i, 2, QTableWidgetItem(self.types.get(topic, '')))
        
        # Resize columns to fit content
        self.table.resizeColumnsToContents()

    def shutdown(self):
        self.timer.stop()
        for sub in self.subscribers.values():
            self.node.destroy_subscription(sub)
        self.subscribers.clear()
        self.msg_times.clear()
        self.types.clear() 