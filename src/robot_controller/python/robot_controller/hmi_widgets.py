"""Presentation-only HMI styling and sections; no state or command handling."""

from PyQt5 import QtWidgets


# Match Item Teach's light touch on native Qt widgets. State/action colors are
# still set by the controller window where their meaning is known.
STATION_STYLE = """
QGroupBox {
    font-weight: 600; border: 1px solid #cbd2da;
    border-radius: 6px; margin-top: 12px; padding: 12px 8px 8px;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 10px; padding: 0 4px;
}
QLineEdit, QComboBox { min-height: 24px; }
QPushButton { min-height: 28px; padding: 2px 8px; }
QPushButton:checked {
    background: #d9eafa; color: #123b60;
    border: 1px solid #4783b5; border-radius: 4px;
}
"""


def section(title):
    panel = QtWidgets.QGroupBox(title)
    panel.setMinimumWidth(0)
    layout = QtWidgets.QVBoxLayout(panel)
    layout.setSpacing(8)
    return panel, layout


def hint(text):
    label = QtWidgets.QLabel(text)
    label.setWordWrap(True)
    label.setMinimumWidth(0)
    return label
