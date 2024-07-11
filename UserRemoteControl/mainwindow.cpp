#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDebug>
#include <QtMath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)

    // battery
    , batterySerial(new QSerialPort(this))
    , batteryTimer(new QTimer(this))
    , maxExpectedPower(6.0)
    , batteryDataBuffer()

    // radar
    , laserActive(false)
    , autoMode(false)
    , r(445.0) // needle radius (pixels)
    , angleOffset(0.05) // needle angle offset (rad)
{
    ui->setupUi(this);

    // Load bg image (radar)
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    pix = QPixmap(":/src/radar.png");
    scene->addPixmap(pix);

    // Set up QSerialPort
    arduino = new QSerialPort;
    arduino_is_available = false;
    radarSerial = "COM9";

    // Initialize needle at 0 degrees
    QPen blackpen(Qt::black);
    QBrush graybrush(Qt::gray);
    t_up = angleOffset;
    t_lo = -angleOffset;
    triangle.append(QPointF(r * qCos(t_up) + 505, -r * qSin(t_up) + 495));
    triangle.append(QPointF(505, 495));
    triangle.append(QPointF(r * qCos(t_lo) + 505, -r * qSin(t_lo) + 495));
    needle = scene->addPolygon(triangle, blackpen, graybrush);
    needle->setOpacity(0.30);

    // Check which port the Arduino is on
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()) {
        if (serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()) {
            if (serialPortInfo.vendorIdentifier() == arduino_uno_vendorID) {
                if (serialPortInfo.productIdentifier() == arduino_uno_productID) {
                    radarSerial = serialPortInfo.portName();
                    arduino_is_available = true;
                    qDebug() << "Port available!";
                }
            }
        }
    }

    // Setup port if available
    if (arduino_is_available) {
        // Open and configure port
        arduino->setPortName(radarSerial);
        arduino->open(QSerialPort::ReadWrite);
        arduino->setBaudRate(QSerialPort::Baud9600);
        arduino->setDataBits(QSerialPort::Data8);
        arduino->setParity(QSerialPort::NoParity);
        arduino->setStopBits(QSerialPort::OneStop);
        arduino->setFlowControl(QSerialPort::NoFlowControl);
        // Slot for updating value
        QObject::connect(arduino, SIGNAL(readyRead()), this, SLOT(readSerial()));
    } else {
        // Give a message
        QMessageBox::warning(this, "Port error", "Couldn't find Arduino");
    }

    autoTimer = new QTimer(this); // Initialize timer
    connect(autoTimer, &QTimer::timeout, this, &MainWindow::updateServoAuto);

    laserTimer = new QTimer(this); // Initialize laser timer
    connect(laserTimer, &QTimer::timeout, this, &MainWindow::handleLaserActivation);

    /* resumeTimer = new QTimer(this); // Timer to resume operations
    connect(resumeTimer, &QTimer::timeout, this, &MainWindow::resumeOperation); */

    // Connect signals and slots
    connect(ui->forwardButton, &QPushButton::pressed, this, &MainWindow::moveForward);
    connect(ui->backwardButton, &QPushButton::pressed, this, &MainWindow::moveBackward);
    connect(ui->leftButton, &QPushButton::pressed, this, &MainWindow::turnLeft);
    connect(ui->rightButton, &QPushButton::pressed, this, &MainWindow::turnRight);

    // Setup serial port
    serial = new QSerialPort(this);
    serial->setPortName("COM3"); // Adjust to your port
    serial->setBaudRate(QSerialPort::Baud9600);
    if (serial->open(QIODevice::ReadWrite)) {
        connect(serial, &QSerialPort::readyRead, this, &MainWindow::updateSensorData);
    } else {
        qDebug() << "Failed to open main serial port.";
    }

    // Setup progress bar
    powerProgressBar = ui->persentase;
    powerProgressBar->setRange(0, 100);
    powerProgressBar->setValue(0);
    powerProgressBar->setTextVisible(true);
    powerProgressBar->setStyleSheet(
        "QProgressBar {"
        "   border: 2px solid grey;"
        "   border-radius: 5px;"
        "   text-align: center;"
        "}"
        "QProgressBar::chunk {"
        "   background-color: #05B8CC;"
        "   width: 20px;"
        "}"
        );

    // Setup battery serial port
    batterySerial->setPortName("COM10"); // Adjust to your port
    batterySerial->setBaudRate(QSerialPort::Baud115200);
    batterySerial->setDataBits(QSerialPort::Data8);
    batterySerial->setParity(QSerialPort::NoParity);
    batterySerial->setStopBits(QSerialPort::OneStop);
    batterySerial->setFlowControl(QSerialPort::NoFlowControl);

    if (batterySerial->open(QIODevice::ReadOnly)) {
        connect(batterySerial, &QSerialPort::readyRead, this, &MainWindow::readBatteryData);
        qDebug() << "Battery serial port opened successfully.";
    } else {
        qDebug() << "Failed to open battery serial port.";
    }

    batteryTimer->start(2000);  // Update battery data every 2 seconds
}

void MainWindow::moveForward() {
    serial->write("FORWARD\n");
}

void MainWindow::moveBackward() {
    serial->write("BACKWARD\n");
}

void MainWindow::turnLeft() {
    serial->write("LEFT\n");
}

void MainWindow::turnRight() {
    serial->write("RIGHT\n");
}

void MainWindow::updateSensorData() {
    QByteArray data = serial->readAll();
    QList<QByteArray> sensorData = data.split(',');

    if (sensorData.size() >= 6) {
        ui->cameraLabel->setText(sensorData.at(0));
        ui->gpsLabel->setText(sensorData.at(2));
        ui->accelerometerLabel->setText(sensorData.at(3));
        ui->imuLabel->setText(sensorData.at(4));
        ui->ultrasonicLabel->setText(sensorData.at(5));
        ui->speedLcd->display(sensorData.at(6).toInt());
    }
}

void MainWindow::readBatteryData() {
    batteryDataBuffer.append(batterySerial->readAll());

    while (batteryDataBuffer.contains('\n')) {
        int newlineIndex = batteryDataBuffer.indexOf('\n');
        QString dataString = batteryDataBuffer.left(newlineIndex).trimmed();
        batteryDataBuffer.remove(0, newlineIndex + 1);

        QStringList values = dataString.split(',');
        if (values.size() >= 3) {
            float power = values.at(2).toFloat();
            int percentage = qRound((power / maxExpectedPower) * 100);
            percentage = qMin(percentage, 100);
            powerProgressBar->setValue(percentage);
            powerProgressBar->setFormat(QString("%1% (%2W / %3W)").arg(percentage).arg(power, 0, 'f', 1).arg(maxExpectedPower, 0, 'f', 1));
        } else {
            qDebug() << "Incomplete battery data received: " << dataString;
        }
    }
}

void MainWindow::readSerial() {
    // Read data from serial port
    serialData.append(arduino->readAll());

    // Check if we have a complete message (terminated by '\n')
    if (serialData.endsWith('\n')) {
        // Process the complete message
        QString dataString = QString::fromStdString(serialData.toStdString()).trimmed();
        serialData.clear();

        qDebug() << "Raw data received:" << dataString; // Debugging: Print raw data

        QStringList dataList = dataString.split(',');

        if (dataList.size() == 2) {
            bool ok1, ok2;
            float angle = dataList[0].toFloat(&ok1);
            float distance = dataList[1].toFloat(&ok2);

            if (ok1 && ok2) {
                qDebug() << "Parsed angle:" << angle;

                // Ensure distance is within 0-500 cm
                if (distance >= 0 && distance <= 500) {
                    qDebug() << "Parsed distance:" << distance;

                    // Update range label in GUI
                    QString rangeText = QString::number(distance, 'f', 2) + " cm";
                    ui->range->setText(rangeText);

                    // Update detection point only if not pausing
                    if (!laserActive) {
                        updateDetectionPoint(angle, distance);
                    }

                    // Check distance for laser activation
                    if (distance < 50 && !laserActive) { // Activate laser if distance < 50 cm
                        laserActive = true;
                        laserTimer->start(2000); // Start laser timer for 2 seconds
                        // Save current auto mode and slider state
                        previousAutoMode = autoMode;
                        previousSliderState = ui->verticalSlider->isEnabled();

                        if (autoMode) {
                            autoTimer->stop(); // Stop auto timer
                        }
                        setSliderEnabled(false); // Disable the slider
                        qDebug() << "Laser turned on"; // Debugging: Print laser on status
                        updateLaserStatus("Laser: On"); // Update GUI to show laser is on
                    }

                    // Update angle label in GUI
                    QString angleText = QString::number(angle, 'd', 0);
                    ui->angle->setText(angleText);
                } else {
                    qDebug() << "Error: Distance out of range" << distance;
                }
            } else {
                qDebug() << "Error: Incorrect data format received" << dataString;
            }
        } else {
            qDebug() << "Error: Incorrect data format received" << dataString;
        }
    }
}

void MainWindow::setSliderEnabled(bool enabled) {
    ui->verticalSlider->setEnabled(enabled);
}

void MainWindow::handleLaserActivation() {
    laserActive = false;
    laserTimer->stop();
    qDebug() << "Laser turned off"; // Debugging: Print laser off status
    updateLaserStatus("Laser: Off"); // Update GUI to show laser is off

    // Restore previous auto mode and slider state
    if (previousAutoMode) {
        autoTimer->start(500); // Resume auto timer
    }
    setSliderEnabled(previousSliderState); // Re-enable the slider if it was previously enabled
}

void MainWindow::updateServo(QString command) {
    if (arduino->isWritable()) {
        arduino->write(command.toStdString().c_str());
    } else {
        qDebug() << "Couldn't write to serial!";
    }
}

void MainWindow::updateServoAuto() {
    static int angle = 0;
    static bool increasing = true;

    if (increasing) {
        angle += 5;
        if (angle >= 180) {
            angle = 180;
            increasing = false;
        }
    } else {
        angle -= 5;
        if (angle <= 0) {
            angle = 0;
            increasing = true;
        }
    }

    updateServo(QString::number(angle) + "\n");
    ui->verticalSlider->setValue(angle); // Update slider position
}

void MainWindow::updateDetectionPoint(float angle, float distance) {
    qDebug() << "Updating detection point at angle:" << angle << "distance:" << distance; // Debugging: Print detection point
    float radAngle = qDegreesToRadians(angle);
    float x = distance * qCos(radAngle);
    float y = distance * qSin(radAngle);

    QGraphicsRectItem* point = scene->addRect(505 + x, 495 - y, 3, 3, QPen(Qt::red), QBrush(Qt::red));
    detectionPoints.append(point);

    // Update the needle position
    float t_up = radAngle + angleOffset;
    float t_lo = radAngle - angleOffset;
    QPolygonF newTriangle;
    newTriangle.append(QPointF(r * qCos(t_up) + 505, -r * qSin(t_up) + 495));
    newTriangle.append(QPointF(505, 495));
    newTriangle.append(QPointF(r * qCos(t_lo) + 505, -r * qSin(t_lo) + 495));
    needle->setPolygon(newTriangle);

    // Clear old detection points
    clearOldDetectionPoints();
}

void MainWindow::clearOldDetectionPoints() {
    // Remove old points from the scene
    while (detectionPoints.size() > 50) { // Keep the last 50 points
        QGraphicsRectItem* point = detectionPoints.takeFirst();
        scene->removeItem(point);
        delete point;
    }
}

void MainWindow::on_button0_clicked() {
    updateServo("0\n");
    ui->verticalSlider->setValue(0);
}

void MainWindow::on_button90_clicked() {
    updateServo("90\n");
    ui->verticalSlider->setValue(90);
}

void MainWindow::on_button180_clicked() {
    updateServo("180\n");
    ui->verticalSlider->setValue(180);
}

void MainWindow::on_verticalSlider_valueChanged(int value) {
    updateServo(QString::number(value) + "\n");
}

void MainWindow::on_button45_clicked() {
    updateServo("45\n");
    ui->verticalSlider->setValue(45);
}

void MainWindow::on_button135_clicked() {
    updateServo("135\n");
    ui->verticalSlider->setValue(135);
}

void MainWindow::on_button_auto_clicked() {
    autoMode = !autoMode;

    if (autoMode) {
        autoTimer->start(500); // Update servo every 500 ms
        ui->button_auto->setText("Stop Auto");
    } else {
        autoTimer->stop();
        ui->button_auto->setText("Start Auto");
    }
}

void MainWindow::resumeOperation() {
    laserActive = false;
    qDebug() << "Resuming normal operation"; // Debugging: Print resume status
    updateLaserStatus("Laser: Off"); // Ensure laser status is off when resuming
}

void MainWindow::updateLaserStatus(const QString &status) {
    ui->textEdit->setPlainText(status);
}

MainWindow::~MainWindow() {
    if (arduino->isOpen()) {
        arduino->close();
    }
    delete ui;
}

