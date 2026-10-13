"""Presentation-only HMI styling and sections; no state or command handling."""

from PyQt5 import QtCore, QtWidgets


STATION_STYLE = """
QWidget#stationHmi, QWidget[role="page"] { background: #edf2f6; }
QLabel { color: #25364a; }
QLabel[role="hint"] { color: #526477; font-size: 12px; }
QGroupBox {
    background: #ffffff; border: 1px solid #d5dfe8; border-radius: 8px;
    margin-top: 14px; padding: 14px 10px 10px; font-weight: 600;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 14px; padding: 0 5px; color: #25364a;
}
QPushButton, QToolButton {
    background: #ffffff; color: #25364a; border: 1px solid #bac8d5;
    border-radius: 5px; padding: 6px 12px;
}
QPushButton:hover, QToolButton:hover { background: #eef5fa; border-color: #557e9f; }
QPushButton:pressed, QToolButton:pressed { background: #dceaf4; }
QPushButton:disabled, QToolButton:disabled {
    color: #7b8793; background: #f0f3f6; border-color: #d9e0e7;
}
QPushButton:focus, QToolButton:focus { border: 2px solid #2775a8; }
QLineEdit {
    background: #ffffff; color: #203247; border: 1px solid #b9c7d4;
    border-radius: 4px; padding: 5px 8px; min-height: 25px;
    selection-background-color: #246b9a;
}
QLineEdit:focus { border-color: #2775a8; }
QLineEdit:disabled {
    background: #f1f4f7; color: #6d7c8c; border-color: #d9e0e7;
}
QTabWidget::pane { border: 0; }
QTabBar::tab {
    min-width: 125px; padding: 10px 18px; color: #526477;
    border-bottom: 3px solid transparent;
}
QTabBar::tab:selected { color: #164d72; border-bottom-color: #246b9a; font-weight: 700; }
QTabBar::tab:hover { background: #e2ebf2; }
QCheckBox { color: #354a5f; spacing: 8px; padding: 4px 0; }
QSlider::groove:horizontal { height: 5px; background: #d5dfe8; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #397aa8; border-radius: 2px; }
QSlider::handle:horizontal {
    width: 16px; margin: -6px 0; background: #ffffff;
    border: 2px solid #397aa8; border-radius: 8px;
}
QSlider::sub-page:horizontal:disabled { background: #aebdca; }
QSlider::handle:horizontal:disabled { border-color: #aebdca; }
QProgressBar { background: #e1e7ed; border: 0; border-radius: 4px; }
QProgressBar::chunk { background: #397aa8; border-radius: 4px; }
QProgressBar::chunk:disabled { background: #aebdca; }
QToolTip { background: #203247; color: #ffffff; padding: 6px; border: 0; }
"""


class ElidedLabel(QtWidgets.QLabel):
    """Keep long recipe names within their card and expose the complete name to AT."""

    def __init__(self):
        super().__init__()
        self.full_text = ''
        self.setTextFormat(QtCore.Qt.PlainText)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)

    def setText(self, text):
        self.full_text = text
        self.setAccessibleDescription(text)
        self._elide()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._elide()

    def _elide(self):
        super().setText(self.fontMetrics().elidedText(
            self.full_text, QtCore.Qt.ElideMiddle, self.contentsRect().width()))


def section(title):
    panel = QtWidgets.QGroupBox(title)
    panel.setMinimumWidth(0)
    layout = QtWidgets.QVBoxLayout(panel)
    layout.setSpacing(8)
    return panel, layout


def hint(text):
    label = QtWidgets.QLabel(text)
    label.setTextFormat(QtCore.Qt.PlainText)
    label.setProperty('role', 'hint')
    label.setWordWrap(True)
    label.setMinimumWidth(0)
    return label
