"""Shared read-only, video-first presentation for platform and bin teaching."""

from python_qt_binding import QtCore, QtGui, QtWidgets


def visual_teach_layout(window, title, video_title, controls, guidance, actions, hint):
    window.resize(1500, 900)
    window.setMinimumSize(1000, 620)
    window.setStyleSheet("""
        QGroupBox { font-weight: 600; border: 1px solid #cbd2da; border-radius: 6px;
                    margin-top: 12px; padding: 12px 8px 8px; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
        QLineEdit, QComboBox { min-height: 24px; }
        QPushButton { min-height: 28px; padding: 2px 8px; }
        QSplitter::handle { background: #cbd2da; }
    """)
    root = QtWidgets.QVBoxLayout(window)
    root.setContentsMargins(12, 10, 12, 10)
    header = QtWidgets.QHBoxLayout()
    heading = QtWidgets.QLabel(title)
    heading.setFont(QtGui.QFont("Sans", 18, QtGui.QFont.Bold))
    header.addWidget(heading)
    header.addWidget(QtWidgets.QLabel("Teach files / read-only TF preview / no robot motion"))
    header.addStretch(1)
    header.addWidget(window.save_button)
    root.addLayout(header)
    split = window.workspace_split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
    split.setChildrenCollapsible(False)
    split.setHandleWidth(6)
    scroll = window.settings_scroll = QtWidgets.QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
    scroll.setMinimumWidth(300)
    scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
    sidebar = QtWidgets.QWidget()
    column = QtWidgets.QVBoxLayout(sidebar)
    column.setContentsMargins(0, 0, 8, 0)
    setup = QtWidgets.QGroupBox("Setup")
    setup.setLayout(controls)
    column.addWidget(setup)
    quick_help = QtWidgets.QLabel(hint)
    quick_help.setWordWrap(True)
    column.addWidget(quick_help)
    details_toggle = window.details_toggle = QtWidgets.QToolButton()
    details_toggle.setText("Calibration / status details")
    details_toggle.setCheckable(True)
    details_toggle.setArrowType(QtCore.Qt.RightArrow)
    details_toggle.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
    column.addWidget(details_toggle)
    details = window.details_panel = QtWidgets.QWidget()
    detail_layout = QtWidgets.QVBoxLayout(details)
    detail_layout.setContentsMargins(0, 0, 0, 0)
    for label in (window.details, window.status, window.output, guidance):
        label.setMinimumHeight(0)
        label.setWordWrap(True)
        detail_layout.addWidget(label)
    details.hide()
    details_toggle.toggled.connect(details.setVisible)
    details_toggle.toggled.connect(lambda on: details_toggle.setArrowType(
        QtCore.Qt.DownArrow if on else QtCore.Qt.RightArrow))
    column.addWidget(details)
    column.addStretch(1)
    scroll.setWidget(sidebar)
    split.addWidget(scroll)
    visual = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(visual)
    layout.setContentsMargins(4, 0, 0, 0)
    row = QtWidgets.QHBoxLayout()
    for button in actions:
        row.addWidget(button)
    layout.addLayout(row)
    caption = QtWidgets.QLabel(video_title)
    caption.setStyleSheet("background: #202a35; color: #e1e7ed; padding: 8px; font-weight: 600;")
    layout.addWidget(caption)
    window.video.setMinimumSize(400, 300)
    window.video.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Expanding)
    window.video.setWordWrap(True)
    layout.addWidget(window.video, 1)
    split.addWidget(visual)
    split.setStretchFactor(0, 0)
    split.setStretchFactor(1, 1)
    split.setSizes([340, 1160])
    root.addWidget(split, 1)
    window.feedback = QtWidgets.QLabel("Select and apply settings to begin.")
    window.feedback.setWordWrap(True)
    root.addWidget(window.feedback)


def update_teach_feedback(window):
    """Keep a compact actionable status visible when expanded details are hidden."""
    full = window.status.text()
    lines = full.splitlines()
    summary = " | ".join(lines[:2])
    window.feedback.setText(summary)
    window.feedback.setToolTip(full)


def paint_teach_gate(image, text):
    painter = QtGui.QPainter(image)
    font = QtGui.QFont("Sans", max(12, image.width() // 95), QtGui.QFont.Bold)
    painter.setFont(font)
    metrics = painter.fontMetrics()
    height = metrics.height() + 16
    top = max(0, image.height() - height)
    painter.fillRect(0, top, image.width(), height, QtGui.QColor(0, 0, 0, 195))
    painter.setPen(QtGui.QColor("white"))
    painter.drawText(10, top + 8 + metrics.ascent(), metrics.elidedText(
        text, QtCore.Qt.ElideRight, max(1, image.width() - 20)))
    painter.end()
