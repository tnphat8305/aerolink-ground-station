#include <QtCore/QDateTime>
#include <QtCore/QDebug>
#include <QtCore/QHash>
#include <QtCore/QMetaObject>
#include <QtCore/QSet>
#include <QtCore/QThread>
#include <QtCore/QTimer>
#include <QtCore/QElapsedTimer>
#include <QtCore/QVariant>
#include <QtCore/QVector>
#include <QtGui/QPainter>
#include <QtGui/QPainterPath>
#include <QtGui/QPen>
#include <QtGui/QMouseEvent>
#include <QtGui/QPixmap>
#include <QtGui/QWheelEvent>
#include <QtNetwork/QNetworkAccessManager>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkRequest>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace {

constexpr uint8_t MavStxV1 = 0xFE;
constexpr uint8_t MavStxV2 = 0xFD;

constexpr uint8_t MavTypeGcs = 6;
constexpr uint8_t MavAutopilotInvalid = 8;
constexpr uint8_t MavStateActive = 4;

constexpr uint8_t MavFrameGlobalRelativeAltInt = 6;
constexpr uint8_t MavModeFlagCustomModeEnabled = 1;

constexpr uint8_t MavDataStreamExtendedStatus = 2;
constexpr uint8_t MavDataStreamPosition = 6;
constexpr uint8_t MavDataStreamExtra1 = 10;
constexpr uint8_t MavDataStreamExtra2 = 11;
constexpr uint8_t MavDataStreamExtra3 = 12;

constexpr uint8_t MspApiVersion = 1;
constexpr uint8_t MspFcVariant = 2;
constexpr uint8_t MspFcVersion = 3;
constexpr uint8_t MspStatus = 101;
constexpr uint8_t MspRawImu = 102;
constexpr uint8_t MspRawGps = 106;
constexpr uint8_t MspAttitude = 108;
constexpr uint8_t MspAltitude = 109;
constexpr uint8_t MspAnalog = 110;
constexpr uint8_t MspBatteryState = 130;

constexpr uint16_t MavCmdNavLand = 21;
constexpr uint16_t MavCmdNavTakeoff = 22;
constexpr uint16_t MavCmdNavReturnToLaunch = 20;
constexpr uint16_t MavCmdComponentArmDisarm = 400;

constexpr uint32_t ArduModeGuided = 4;
constexpr uint32_t ArduModeRtl = 6;

template <typename T>
T readLe(const QByteArray &data, int offset)
{
    T value{};
    if (offset < 0 || offset + int(sizeof(T)) > data.size()) {
        return value;
    }
    std::memcpy(&value, data.constData() + offset, sizeof(T));
    return value;
}

template <typename T>
void writeLe(QByteArray &data, int offset, T value)
{
    if (offset < 0 || offset + int(sizeof(T)) > data.size()) {
        return;
    }
    std::memcpy(data.data() + offset, &value, sizeof(T));
}

void crcAccumulate(uint8_t value, uint16_t &crc)
{
    value ^= uint8_t(crc & 0xFF);
    value ^= uint8_t(value << 4);
    crc = uint16_t((crc >> 8) ^ (uint16_t(value) << 8) ^ (uint16_t(value) << 3) ^ (uint16_t(value) >> 4));
}

uint16_t mavCrc(const QByteArray &headerWithoutStx, const QByteArray &payload, uint8_t crcExtra)
{
    uint16_t crc = 0xFFFF;
    for (char b : headerWithoutStx) {
        crcAccumulate(uint8_t(b), crc);
    }
    for (char b : payload) {
        crcAccumulate(uint8_t(b), crc);
    }
    crcAccumulate(crcExtra, crc);
    return crc;
}

uint8_t crcExtra(uint32_t msgId)
{
    switch (msgId) {
    case 0: return 50;   // HEARTBEAT
    case 1: return 124;  // SYS_STATUS
    case 11: return 89;  // SET_MODE
    case 24: return 24;  // GPS_RAW_INT
    case 30: return 39;  // ATTITUDE
    case 33: return 104; // GLOBAL_POSITION_INT
    case 66: return 148; // REQUEST_DATA_STREAM
    case 76: return 152; // COMMAND_LONG
    case 77: return 143; // COMMAND_ACK
    case 86: return 5;   // SET_POSITION_TARGET_GLOBAL_INT
    case 109: return 185; // RADIO_STATUS
    case 253: return 83; // STATUSTEXT
    default: return 0;
    }
}

QString arduModeName(uint32_t customMode)
{
    switch (customMode) {
    case 0: return "STABILIZE";
    case 1: return "ACRO";
    case 2: return "ALT_HOLD";
    case 3: return "AUTO";
    case 4: return "GUIDED";
    case 5: return "LOITER";
    case 6: return "RTL";
    case 7: return "CIRCLE";
    case 9: return "LAND";
    case 11: return "DRIFT";
    case 13: return "SPORT";
    case 14: return "FLIP";
    case 15: return "AUTOTUNE";
    case 16: return "POSHOLD";
    case 17: return "BRAKE";
    case 18: return "THROW";
    case 19: return "AVOID_ADSB";
    case 20: return "GUIDED_NOGPS";
    case 21: return "SMART_RTL";
    default: return QString("MODE %1").arg(customMode);
    }
}

struct MavPacket {
    uint32_t msgId = 0;
    uint8_t sysId = 0;
    uint8_t compId = 0;
    QByteArray payload;
};

struct MspPacket {
    uint8_t cmd = 0;
    bool error = false;
    QByteArray payload;
};

class MavParser {
public:
    QList<MavPacket> push(const QByteArray &bytes)
    {
        buffer_.append(bytes);
        if (buffer_.size() > maxBufferSize_) {
            buffer_.remove(0, buffer_.size() - maxBufferSize_);
        }

        QList<MavPacket> packets;

        while (!buffer_.isEmpty()) {
            const uint8_t stx = uint8_t(buffer_.at(0));
            if (stx != MavStxV1 && stx != MavStxV2) {
                buffer_.remove(0, 1);
                continue;
            }

            if (stx == MavStxV1) {
                if (buffer_.size() < 8) {
                    break;
                }
                const int payloadLen = uint8_t(buffer_.at(1));
                const int frameLen = 6 + payloadLen + 2;
                if (buffer_.size() < frameLen) {
                    break;
                }

                QByteArray frame = buffer_.left(frameLen);
                buffer_.remove(0, frameLen);

                const uint32_t msgId = uint8_t(frame.at(5));
                const QByteArray header = frame.mid(1, 5);
                const QByteArray payload = frame.mid(6, payloadLen);
                const uint16_t expected = readLe<uint16_t>(frame, 6 + payloadLen);
                if (mavCrc(header, payload, crcExtra(msgId)) != expected) {
                    continue;
                }

                packets.append({msgId, uint8_t(frame.at(3)), uint8_t(frame.at(4)), payload});
                continue;
            }

            if (buffer_.size() < 12) {
                break;
            }
            const int payloadLen = uint8_t(buffer_.at(1));
            const bool signedFrame = (uint8_t(buffer_.at(2)) & 0x01) != 0;
            const int signatureLen = signedFrame ? 13 : 0;
            const int frameLen = 10 + payloadLen + 2 + signatureLen;
            if (buffer_.size() < frameLen) {
                break;
            }

            QByteArray frame = buffer_.left(frameLen);
            buffer_.remove(0, frameLen);

            const uint32_t msgId = uint8_t(frame.at(7)) | (uint32_t(uint8_t(frame.at(8))) << 8) | (uint32_t(uint8_t(frame.at(9))) << 16);
            const QByteArray header = frame.mid(1, 9);
            const QByteArray payload = frame.mid(10, payloadLen);
            const uint16_t expected = readLe<uint16_t>(frame, 10 + payloadLen);
            if (mavCrc(header, payload, crcExtra(msgId)) != expected) {
                continue;
            }

            packets.append({msgId, uint8_t(frame.at(5)), uint8_t(frame.at(6)), payload});
        }

        return packets;
    }

    void clear()
    {
        buffer_.clear();
    }

private:
    static constexpr int maxBufferSize_ = 8192;
    QByteArray buffer_;
};

class MspParser {
public:
    QList<MspPacket> push(const QByteArray &bytes)
    {
        buffer_.append(bytes);
        if (buffer_.size() > maxBufferSize_) {
            buffer_.remove(0, buffer_.size() - maxBufferSize_);
        }

        QList<MspPacket> packets;

        while (buffer_.size() >= 6) {
            if (uint8_t(buffer_.at(0)) != '$') {
                buffer_.remove(0, 1);
                continue;
            }

            if (buffer_.size() < 3) {
                break;
            }

            if (uint8_t(buffer_.at(1)) != 'M') {
                buffer_.remove(0, 1);
                continue;
            }

            const uint8_t direction = uint8_t(buffer_.at(2));
            if (direction != '>' && direction != '!') {
                buffer_.remove(0, 1);
                continue;
            }

            const int payloadLen = uint8_t(buffer_.at(3));
            const int frameLen = 6 + payloadLen;
            if (buffer_.size() < frameLen) {
                break;
            }

            QByteArray frame = buffer_.left(frameLen);
            buffer_.remove(0, frameLen);

            uint8_t checksum = uint8_t(frame.at(3)) ^ uint8_t(frame.at(4));
            const QByteArray payload = frame.mid(5, payloadLen);
            for (char b : payload) {
                checksum ^= uint8_t(b);
            }

            if (checksum != uint8_t(frame.at(5 + payloadLen))) {
                continue;
            }

            packets.append({uint8_t(frame.at(4)), direction == '!', payload});
        }

        return packets;
    }

    void clear()
    {
        buffer_.clear();
    }

private:
    static constexpr int maxBufferSize_ = 8192;
    QByteArray buffer_;
};

QByteArray makeV1Packet(uint8_t &seq, uint8_t sysId, uint8_t compId, uint8_t msgId, const QByteArray &payload)
{
    QByteArray header;
    header.append(char(payload.size()));
    header.append(char(seq++));
    header.append(char(sysId));
    header.append(char(compId));
    header.append(char(msgId));

    const uint16_t crc = mavCrc(header, payload, crcExtra(msgId));

    QByteArray packet;
    packet.append(char(MavStxV1));
    packet.append(header);
    packet.append(payload);
    packet.append(char(crc & 0xFF));
    packet.append(char((crc >> 8) & 0xFF));
    return packet;
}

QByteArray makeMspRequest(uint8_t cmd, const QByteArray &payload = QByteArray())
{
    QByteArray packet;
    packet.append('$');
    packet.append('M');
    packet.append('<');
    packet.append(char(payload.size()));
    packet.append(char(cmd));

    uint8_t checksum = uint8_t(payload.size()) ^ cmd;
    for (char b : payload) {
        checksum ^= uint8_t(b);
        packet.append(b);
    }

    packet.append(char(checksum));
    return packet;
}

} // namespace

class DroneMapWidget : public QWidget {
public:
    explicit DroneMapWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(420);
        setMouseTracking(true);
        network_ = new QNetworkAccessManager(this);
        clockTimer_ = new QTimer(this);
        clockTimer_->setInterval(1000);
        connect(clockTimer_, &QTimer::timeout, this, [this]() {
            update();
        });
        clockTimer_->start();
        centerLat_ = 10.7769;
        centerLon_ = 106.7009;
        hasCenter_ = true;
    }

    void updatePosition(double lat, double lon, double alt)
    {
        currentLat_ = lat;
        currentLon_ = lon;
        currentAlt_ = alt;
        const bool firstDronePosition = !hasPosition_;
        hasPosition_ = true;
        if (!hasHome_) {
            homeLat_ = lat;
            homeLon_ = lon;
            hasHome_ = true;
        }
        if (firstDronePosition || !hasCenter_) {
            centerLat_ = lat;
            centerLon_ = lon;
            hasCenter_ = true;
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), QColor("#0B1324"));

        const QRectF mapRect = rect().adjusted(20, 20, -20, -50);
        p.setPen(QPen(QColor("#243653"), 1));
        p.setBrush(Qt::NoBrush);
        p.drawRoundedRect(mapRect, 8, 8);

        drawOsmTiles(p, mapRect);

        if (!hasPosition_) {
            p.setPen(QColor("#6F819F"));
            p.drawText(mapRect, Qt::AlignCenter, "OSM map is ready. Waiting for drone GPS position...");
            p.setPen(QColor("#D9E7FF"));
            const QString info = QString("OSM tiles | Zoom %1 | Default center: Ho Chi Minh City | Use mouse wheel to zoom, drag to pan")
                                     .arg(zoom_);
            p.drawText(rect().adjusted(24, 0, -24, -16), Qt::AlignBottom | Qt::AlignLeft, info);
            return;
        }

        if (hasHome_) {
            const QPointF home = geoToWidget(homeLat_, homeLon_, mapRect);
            p.setPen(QPen(QColor("#56CCF2"), 2));
            p.setBrush(QColor(16, 38, 30, 210));
            p.drawEllipse(home, 7, 7);
            p.drawText(home + QPointF(10, -10), "HOME");
        }

        const QPointF drone = geoToWidget(currentLat_, currentLon_, mapRect);
        p.setPen(QPen(QColor("#6EE7B7"), 2));
        p.setBrush(QColor("#10B981"));
        p.drawEllipse(drone, 9, 9);
        p.drawText(drone + QPointF(12, -12), "DRONE");

        p.setPen(QColor("#D9E7FF"));
        const QString info = QString("OSM tiles | Zoom %1 | Lat %2 | Lon %3 | Alt %4 m")
                                 .arg(zoom_)
                                 .arg(currentLat_, 0, 'f', 7)
                                 .arg(currentLon_, 0, 'f', 7)
                                 .arg(currentAlt_, 0, 'f', 1);
        p.drawText(rect().adjusted(24, 0, -24, -16), Qt::AlignBottom | Qt::AlignLeft, info);
        drawClock(p, mapRect);
    }

    void wheelEvent(QWheelEvent *event) override
    {
        const int delta = event->angleDelta().y();
        if (delta > 0 && zoom_ < 19) {
            ++zoom_;
        } else if (delta < 0 && zoom_ > 3) {
            --zoom_;
        }
        update();
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            dragging_ = true;
            lastMousePos_ = event->pos();
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (!dragging_) {
            return;
        }

        const QPoint delta = event->pos() - lastMousePos_;
        lastMousePos_ = event->pos();

        QPointF centerWorld = geoToWorld(centerLat_, centerLon_, zoom_);
        centerWorld -= QPointF(delta.x(), delta.y());
        const QPointF geo = worldToGeo(centerWorld, zoom_);
        centerLat_ = geo.x();
        centerLon_ = geo.y();
        update();
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            dragging_ = false;
        }
    }

private:
    QString tileKey(int z, int x, int y) const
    {
        return QString("%1/%2/%3").arg(z).arg(x).arg(y);
    }

    QPointF geoToWorld(double lat, double lon, int zoom) const
    {
        const double n = double(1 << zoom);
        const double latRad = lat * 3.14159265358979323846 / 180.0;
        const double x = (lon + 180.0) / 360.0 * n * tileSize_;
        const double y = (1.0 - std::log(std::tan(latRad) + 1.0 / std::cos(latRad)) / 3.14159265358979323846) / 2.0 * n * tileSize_;
        return QPointF(x, y);
    }

    QPointF worldToGeo(const QPointF &world, int zoom) const
    {
        const double n = double(1 << zoom);
        const double x = world.x() / tileSize_;
        const double y = world.y() / tileSize_;
        const double lon = x / n * 360.0 - 180.0;
        const double latRad = std::atan(std::sinh(3.14159265358979323846 * (1.0 - 2.0 * y / n)));
        const double lat = latRad * 180.0 / 3.14159265358979323846;
        return QPointF(lat, lon);
    }

    QPointF geoToWidget(double lat, double lon, const QRectF &mapRect) const
    {
        const QPointF centerWorld = geoToWorld(centerLat_, centerLon_, zoom_);
        const QPointF pointWorld = geoToWorld(lat, lon, zoom_);
        return mapRect.center() + (pointWorld - centerWorld);
    }

    void drawOsmTiles(QPainter &p, const QRectF &mapRect)
    {
        const QPointF centerWorld = geoToWorld(centerLat_, centerLon_, zoom_);
        const QPointF topLeftWorld = centerWorld - QPointF(mapRect.width() / 2.0, mapRect.height() / 2.0);
        const int minTileX = int(std::floor(topLeftWorld.x() / tileSize_));
        const int minTileY = int(std::floor(topLeftWorld.y() / tileSize_));
        const int maxTileX = int(std::floor((topLeftWorld.x() + mapRect.width()) / tileSize_));
        const int maxTileY = int(std::floor((topLeftWorld.y() + mapRect.height()) / tileSize_));
        const int tileCount = 1 << zoom_;

        p.save();
        p.setClipRect(mapRect);
        for (int ty = minTileY; ty <= maxTileY; ++ty) {
            if (ty < 0 || ty >= tileCount) {
                continue;
            }
            for (int tx = minTileX; tx <= maxTileX; ++tx) {
                const int wrappedX = ((tx % tileCount) + tileCount) % tileCount;
                const QString key = tileKey(zoom_, wrappedX, ty);
                requestTile(zoom_, wrappedX, ty);
                const QPointF drawAt(mapRect.left() + tx * tileSize_ - topLeftWorld.x(),
                                     mapRect.top() + ty * tileSize_ - topLeftWorld.y());
                if (tileCache_.contains(key)) {
                    p.drawPixmap(QRectF(drawAt, QSizeF(tileSize_, tileSize_)), tileCache_[key], QRectF(0, 0, tileSize_, tileSize_));
                } else {
                    p.fillRect(QRectF(drawAt, QSizeF(tileSize_, tileSize_)), QColor("#0D1528"));
                    p.setPen(QColor("#243653"));
                    p.drawRect(QRectF(drawAt, QSizeF(tileSize_, tileSize_)));
                }
            }
        }
        p.restore();

        p.setPen(QPen(QColor(9, 15, 29, 180), 3));
        p.drawRoundedRect(mapRect, 8, 8);
        drawClock(p, mapRect);
    }

    void drawClock(QPainter &p, const QRectF &mapRect)
    {
        const QString timeText = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
        const QFontMetrics fm(p.font());
        const QRectF box(mapRect.right() - fm.horizontalAdvance(timeText) - 34, mapRect.top() + 12,
                         fm.horizontalAdvance(timeText) + 22, 30);
        p.setPen(QPen(QColor("#314462"), 1));
        p.setBrush(QColor(11, 19, 36, 220));
        p.drawRoundedRect(box, 6, 6);
        p.setPen(QColor("#E5F0FF"));
        p.drawText(box, Qt::AlignCenter, timeText);
    }

    void requestTile(int z, int x, int y)
    {
        const QString key = tileKey(z, x, y);
        if (tileCache_.contains(key) || pendingTiles_.contains(key)) {
            return;
        }

        pendingTiles_.insert(key);
        QNetworkRequest request(QUrl(QString("https://tile.openstreetmap.org/%1/%2/%3.png").arg(z).arg(x).arg(y)));
        request.setRawHeader("User-Agent", "AeroLinkDroneGroundStation/1.0");
        QNetworkReply *reply = network_->get(request);
        connect(reply, &QNetworkReply::finished, this, [this, reply, key]() {
            pendingTiles_.remove(key);
            if (reply->error() == QNetworkReply::NoError) {
                QPixmap pixmap;
                pixmap.loadFromData(reply->readAll(), "PNG");
                if (!pixmap.isNull()) {
                    tileCache_.insert(key, pixmap);
                    update();
                }
            }
            reply->deleteLater();
        });
    }

    bool hasHome_ = false;
    bool hasPosition_ = false;
    bool hasCenter_ = false;
    bool dragging_ = false;
    double homeLat_ = 0.0;
    double homeLon_ = 0.0;
    double centerLat_ = 0.0;
    double centerLon_ = 0.0;
    double currentLat_ = 0.0;
    double currentLon_ = 0.0;
    double currentAlt_ = 0.0;
    int zoom_ = 17;
    const int tileSize_ = 256;
    QPoint lastMousePos_;
    QNetworkAccessManager *network_ = nullptr;
    QTimer *clockTimer_ = nullptr;
    QHash<QString, QPixmap> tileCache_;
    QSet<QString> pendingTiles_;
};

class AttitudeChartWidget : public QWidget {
public:
    explicit AttitudeChartWidget(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(500);
        clockTimer_ = new QTimer(this);
        clockTimer_->setInterval(1000);
        connect(clockTimer_, &QTimer::timeout, this, [this]() {
            update();
        });
        clockTimer_->start();
    }

    void addSample(double roll, double pitch, double yaw)
    {
        appendLimited(roll_, roll);
        appendLimited(pitch_, pitch);
        appendLimited(yaw_, normalizeAngle(yaw));
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), QColor("#0B1324"));

        const QRectF area = rect().adjusted(36, 24, -24, -28);
        const double gap = 26.0;
        const double panelHeight = (area.height() - gap * 2.0) / 3.0;

        drawPanel(p, QRectF(area.left(), area.top(), area.width(), panelHeight),
                  "ROLL", roll_, QColor("#56CCF2"));
        drawPanel(p, QRectF(area.left(), area.top() + panelHeight + gap, area.width(), panelHeight),
                  "PITCH", pitch_, QColor("#6EE7B7"));
        drawPanel(p, QRectF(area.left(), area.top() + (panelHeight + gap) * 2.0, area.width(), panelHeight),
                  "YAW", yaw_, QColor("#FBBF24"));
        drawClock(p, area);
    }

private:
    void appendLimited(QVector<double> &series, double value)
    {
        series.append(value);
        constexpr int maxSamples = 240;
        while (series.size() > maxSamples) {
            series.removeFirst();
        }
    }

    double normalizeAngle(double value) const
    {
        while (value > 180.0) {
            value -= 360.0;
        }
        while (value < -180.0) {
            value += 360.0;
        }
        return value;
    }

    double seriesRange(const QVector<double> &series) const
    {
        double maxAbs = 45.0;
        for (double value : series) {
            maxAbs = std::max(maxAbs, std::abs(value));
        }
        return std::min(180.0, std::ceil(maxAbs / 15.0) * 15.0);
    }

    void drawPanel(QPainter &p, const QRectF &panel, const QString &title, const QVector<double> &series, const QColor &color)
    {
        p.setPen(QPen(QColor("#243653"), 1));
        p.setBrush(QColor("#0D1528"));
        p.drawRoundedRect(panel, 8, 8);

        const QRectF chart = panel.adjusted(58, 38, -20, -26);
        p.setPen(QPen(QColor("#1D2B44"), 1));
        for (int i = 1; i < 4; ++i) {
            const double y = chart.top() + chart.height() * i / 4.0;
            p.drawLine(QPointF(chart.left(), y), QPointF(chart.right(), y));
        }
        for (int i = 1; i < 8; ++i) {
            const double x = chart.left() + chart.width() * i / 8.0;
            p.drawLine(QPointF(x, chart.top()), QPointF(x, chart.bottom()));
        }

        p.setPen(QPen(QColor("#35537A"), 1));
        p.drawLine(QPointF(chart.left(), chart.center().y()), QPointF(chart.right(), chart.center().y()));

        const double range = seriesRange(series);
        p.setPen(color);
        const QString current = series.isEmpty() ? "--" : QString("%1 deg").arg(series.last(), 0, 'f', 1);
        p.drawText(panel.adjusted(16, 8, -16, -8), Qt::AlignTop | Qt::AlignLeft,
                   QString("%1   %2   range +/- %3 deg").arg(title, current).arg(range, 0, 'f', 0));

        drawSeries(p, chart, series, color, range);
    }

    void drawClock(QPainter &p, const QRectF &area)
    {
        const QString timeText = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
        const QFontMetrics fm(p.font());
        const QRectF box(area.right() - fm.horizontalAdvance(timeText) - 28, area.top() - 2,
                         fm.horizontalAdvance(timeText) + 22, 30);
        p.setPen(QPen(QColor("#314462"), 1));
        p.setBrush(QColor(11, 19, 36, 220));
        p.drawRoundedRect(box, 6, 6);
        p.setPen(QColor("#E5F0FF"));
        p.drawText(box, Qt::AlignCenter, timeText);
    }

    void drawSeries(QPainter &p, const QRectF &chart, const QVector<double> &series, const QColor &color, double range)
    {
        if (series.size() < 2) {
            return;
        }
        QPainterPath path;
        const int n = series.size();
        for (int i = 0; i < n; ++i) {
            const double x = chart.left() + chart.width() * i / double(std::max(1, n - 1));
            const double y = chart.center().y() - (series[i] / range) * (chart.height() / 2.0);
            if (i == 0) {
                path.moveTo(x, y);
            } else {
                path.lineTo(x, y);
            }
        }
        p.setPen(QPen(color, 2));
        p.save();
        p.setClipRect(chart.adjusted(0, -1, 0, 1));
        p.drawPath(path);
        p.restore();
    }

    QVector<double> roll_;
    QVector<double> pitch_;
    QVector<double> yaw_;
    QTimer *clockTimer_ = nullptr;
};

enum class LinkProtocol {
    Unknown,
    Mavlink,
    Msp
};

class MavlinkWorker : public QObject {
    Q_OBJECT

public:
    explicit MavlinkWorker(QObject *parent = nullptr)
        : QObject(parent),
          serial_(new QSerialPort(this)),
          heartbeatTimer_(new QTimer(this)),
          heartbeatValidationTimer_(new QTimer(this)),
          gcsHeartbeatTimer_(new QTimer(this)),
          mspPollTimer_(new QTimer(this)),
          reconnectTimer_(new QTimer(this)),
          linkStatsTimer_(new QTimer(this))
    {
        heartbeatTimer_->setSingleShot(true);
        heartbeatTimer_->setInterval(10000);
        heartbeatValidationTimer_->setSingleShot(true);
        heartbeatValidationTimer_->setInterval(500);
        gcsHeartbeatTimer_->setInterval(1000);
        mspPollTimer_->setInterval(100);
        reconnectTimer_->setInterval(2000);
        linkStatsTimer_->setInterval(1000);
        linkClock_.start();

        connect(serial_, &QSerialPort::readyRead, this, &MavlinkWorker::readSerial);
        connect(serial_, &QSerialPort::errorOccurred, this, &MavlinkWorker::onSerialError);
        connect(heartbeatTimer_, &QTimer::timeout, this, [this]() {
            if (serial_->isOpen() && !heartbeatSeen_) {
                failHeartbeatCheck();
            }
        });
        connect(heartbeatValidationTimer_, &QTimer::timeout, this, [this]() {
            acceptHeartbeatConnection();
        });
        connect(gcsHeartbeatTimer_, &QTimer::timeout, this, &MavlinkWorker::sendGcsHeartbeat);
        connect(mspPollTimer_, &QTimer::timeout, this, &MavlinkWorker::pollMsp);
        connect(reconnectTimer_, &QTimer::timeout, this, &MavlinkWorker::tryReconnect);
        connect(linkStatsTimer_, &QTimer::timeout, this, &MavlinkWorker::publishLinkStats);
        linkStatsTimer_->start();
    }

    bool isConnected() const { return serial_->isOpen(); }

    void connectToVehicle(const QString &port, int baud)
    {
        if (serial_->isOpen()) {
            return;
        }

        port_ = port;
        baud_ = baud;
        reconnectAttempts_ = 0;
        heartbeatSeen_ = false;
        heartbeatAccepted_ = false;
        mspAccepted_ = false;
        protocol_ = LinkProtocol::Unknown;
        baudAdvertised_ = -1;
        mspPollTick_ = 0;
        mspIdentityLogged_ = false;
        mspApiVersion_.clear();
        mspFcVariant_.clear();
        mspFcVersion_.clear();
        wantedOpen_ = true;

        serial_->setPortName(port_);
        serial_->setBaudRate(baud_);
        serial_->setDataBits(QSerialPort::Data8);
        serial_->setParity(QSerialPort::NoParity);
        serial_->setStopBits(QSerialPort::OneStop);
        serial_->setFlowControl(QSerialPort::NoFlowControl);

        if (!serial_->open(QIODevice::ReadWrite)) {
            emit statusChanged(QString("Connection error: %1").arg(serial_->errorString()), "status-disconnected");
            emit disconnected();
            return;
        }
        serial_->setDataTerminalReady(true);
        serial_->setRequestToSend(true);

        parser_.clear();
        mspParser_.clear();
        lastHeartbeatMs_ = -1;
        lastMessageMs_ = -1;
        emit statusChanged(QString("Waiting for MAVLink or MSP at %1 baud...").arg(baud_), "status-warning");
        emit eventLog(QString("Serial opened at %1 baud. Probing MAVLink and MSP...").arg(baud_), "info");
        sendGcsHeartbeat();
        gcsHeartbeatTimer_->start();
        sendMspRequest(MspApiVersion);
        sendMspRequest(MspFcVariant);
        sendMspRequest(MspFcVersion);
        mspPollTimer_->start();
        heartbeatTimer_->start();
    }

    void disconnectFromVehicle()
    {
        reconnectTimer_->stop();
        heartbeatTimer_->stop();
        heartbeatValidationTimer_->stop();
        gcsHeartbeatTimer_->stop();
        mspPollTimer_->stop();
        wantedOpen_ = false;
        if (serial_->isOpen()) {
            serial_->close();
        }
        emit disconnected();
        emit statusChanged("Disconnected", "status-disconnected");
    }

    void changeBaudRate(int baud)
    {
        if (baud <= 0) {
            emit statusChanged("Invalid baud rate", "status-warning");
            emit eventLog("Rejected invalid baud rate.", "warn");
            return;
        }

        baud_ = baud;
        parser_.clear();
        mspParser_.clear();
        heartbeatSeen_ = false;
        heartbeatAccepted_ = false;
        mspAccepted_ = false;
        protocol_ = LinkProtocol::Unknown;
        baudAdvertised_ = -1;
        mspPollTick_ = 0;
        mspIdentityLogged_ = false;
        mspApiVersion_.clear();
        mspFcVariant_.clear();
        mspFcVersion_.clear();
        lastHeartbeatMs_ = -1;
        lastMessageMs_ = -1;

        if (!serial_->isOpen()) {
            emit statusChanged(QString("Baud rate set to %1").arg(baud_), "status-warning");
            emit eventLog(QString("Baud rate prepared: %1").arg(baud_), "info");
            return;
        }

        if (!serial_->setBaudRate(baud_)) {
            emit statusChanged(QString("Failed to apply baud rate: %1").arg(serial_->errorString()), "status-warning");
            emit eventLog(QString("Failed to apply baud rate %1: %2").arg(baud_).arg(serial_->errorString()), "warn");
            return;
        }

        emit statusChanged(QString("Baud rate applied: %1").arg(baud_), "status-warning");
        emit eventLog(QString("Serial baud rate changed live to %1. Probing MAVLink and MSP...").arg(baud_), "info");
        sendGcsHeartbeat();
        sendMspRequest(MspApiVersion);
        mspPollTimer_->start();
        heartbeatTimer_->start();
    }

    void arm()
    {
        try {
            sendCommandLong(MavCmdComponentArmDisarm, 1.0f);
        } catch (const std::exception &e) {
            emit statusChanged(QString("Arm error: %1").arg(e.what()), "status-warning");
        }
    }

    void disarm()
    {
        try {
            sendCommandLong(MavCmdComponentArmDisarm, 0.0f);
        } catch (const std::exception &e) {
            emit statusChanged(QString("Disarm error: %1").arg(e.what()), "status-warning");
        }
    }

    void takeoff(double altitudeM)
    {
        try {
            sendCommandLong(MavCmdNavTakeoff, 0, 0, 0, 0, 0, 0, float(altitudeM));
        } catch (const std::exception &e) {
            emit statusChanged(QString("Takeoff error: %1").arg(e.what()), "status-warning");
        }
    }

    void land()
    {
        try {
            sendCommandLong(MavCmdNavLand);
        } catch (const std::exception &e) {
            emit statusChanged(QString("Land error: %1").arg(e.what()), "status-warning");
        }
    }

    void rth()
    {
        try {
            setMode(ArduModeRtl);
            sendCommandLong(MavCmdNavReturnToLaunch);
        } catch (const std::exception &e) {
            emit statusChanged(QString("RTH error: %1").arg(e.what()), "status-warning");
        }
    }

    void goTo(double lat, double lon, double alt)
    {
        try {
            setMode(ArduModeGuided);

            QByteArray payload(53, char(0));
            writeLe<uint32_t>(payload, 0, 0);
            writeLe<int32_t>(payload, 4, int32_t(lat * 1e7));
            writeLe<int32_t>(payload, 8, int32_t(lon * 1e7));
            writeLe<float>(payload, 12, float(alt));
            writeLe<uint16_t>(payload, 48, 0b0000111111111000);
            payload[50] = char(targetSystem_);
            payload[51] = char(targetComponent_);
            payload[52] = char(MavFrameGlobalRelativeAltInt);
            writePacket(86, payload);
            emit statusChanged("Navigating to target", "status-connected");
        } catch (const std::exception &e) {
            emit statusChanged(QString("Goto error: %1").arg(e.what()), "status-warning");
        }
    }

signals:
    void statusChanged(const QString &text, const QString &style);
    void telemetryUpdate(const QVariantMap &data);
    void linkStatsUpdate(const QVariantMap &data);
    void eventLog(const QString &text, const QString &level);
    void connected();
    void disconnected();
    void connectionLost();

private slots:
    void readSerial()
    {
        const QByteArray bytes = serial_->readAll();
        bytesReceived_ += uint64_t(bytes.size());
        bytesReceivedThisSecond_ += uint64_t(bytes.size());

        const QList<MavPacket> packets = parser_.push(bytes);
        const QList<MspPacket> mspPackets = mspParser_.push(bytes);
        const uint64_t packetCount = uint64_t(packets.size() + mspPackets.size());
        packetsReceived_ += packetCount;
        packetsReceivedThisSecond_ += packetCount;

        for (const MavPacket &packet : packets) {
            handlePacket(packet);
        }
        for (const MspPacket &packet : mspPackets) {
            handleMspPacket(packet);
        }
    }

    void onSerialError(QSerialPort::SerialPortError error)
    {
        if (error == QSerialPort::NoError || error == QSerialPort::TimeoutError || !wantedOpen_) {
            return;
        }
        if (serial_->isOpen()) {
            serial_->close();
        }
        startReconnect();
    }

    void tryReconnect()
    {
        if (serial_->isOpen()) {
            reconnectTimer_->stop();
            return;
        }
        if (reconnectAttempts_ >= maxReconnectAttempts_) {
            reconnectTimer_->stop();
            wantedOpen_ = false;
            emit disconnected();
            emit statusChanged("Connection lost", "status-disconnected");
            return;
        }

        ++reconnectAttempts_;
        emit statusChanged(QString("Reconnecting... (%1/%2)").arg(reconnectAttempts_).arg(maxReconnectAttempts_), "status-warning");

        serial_->setPortName(port_);
        serial_->setBaudRate(baud_);
        heartbeatSeen_ = false;
        mspAccepted_ = false;
        protocol_ = LinkProtocol::Unknown;
        mspPollTick_ = 0;
        mspIdentityLogged_ = false;
        mspApiVersion_.clear();
        mspFcVariant_.clear();
        mspFcVersion_.clear();
        if (serial_->open(QIODevice::ReadWrite)) {
            serial_->setDataTerminalReady(true);
            serial_->setRequestToSend(true);
            reconnectTimer_->stop();
            sendGcsHeartbeat();
            gcsHeartbeatTimer_->start();
            sendMspRequest(MspApiVersion);
            mspPollTimer_->start();
            heartbeatTimer_->start();
        }
    }

private:
    void failHeartbeatCheck()
    {
        heartbeatTimer_->stop();
        heartbeatValidationTimer_->stop();
        gcsHeartbeatTimer_->stop();
        mspPollTimer_->stop();
        reconnectTimer_->stop();
        wantedOpen_ = false;
        parser_.clear();
        heartbeatSeen_ = false;
        heartbeatAccepted_ = false;

        if (serial_->isOpen()) {
            serial_->close();
        }

        emit statusChanged(QString("No MAVLink/MSP response at %1 baud").arg(baud_), "status-disconnected");
        emit eventLog(QString("Connection rejected: no MAVLink HEARTBEAT or MSP response at %1 baud. Check COM port, baud rate, USB cable, or another app holding the port.").arg(baud_), "warn");
        emit disconnected();
    }

    void failBaudMismatch()
    {
        heartbeatTimer_->stop();
        heartbeatValidationTimer_->stop();
        gcsHeartbeatTimer_->stop();
        mspPollTimer_->stop();
        reconnectTimer_->stop();
        wantedOpen_ = false;
        parser_.clear();
        heartbeatSeen_ = false;
        heartbeatAccepted_ = false;

        if (serial_->isOpen()) {
            serial_->close();
        }

        emit statusChanged(QString("Baud mismatch: app %1, source %2").arg(baud_).arg(baudAdvertised_), "status-disconnected");
        emit eventLog(QString("Connection rejected: selected baud %1 does not match test source baud %2.").arg(baud_).arg(baudAdvertised_), "warn");
        emit disconnected();
    }

    void acceptHeartbeatConnection()
    {
        if (!serial_->isOpen() || !heartbeatSeen_ || heartbeatAccepted_) {
            return;
        }
        if (baudAdvertised_ > 0 && baudAdvertised_ != baud_) {
            failBaudMismatch();
            return;
        }

        heartbeatValidationTimer_->stop();
        heartbeatAccepted_ = true;
        protocol_ = LinkProtocol::Mavlink;
        mspPollTimer_->stop();
        reconnectAttempts_ = 0;
        emit statusChanged("Connected", "status-connected");
        if (baudAdvertised_ > 0) {
            emit eventLog(QString("Heartbeat accepted. Test source baud verified: %1.").arg(baudAdvertised_), "info");
        } else {
            emit eventLog("Heartbeat accepted. MAVLink link is active.", "info");
        }
        requestTelemetryStreams();
        emit connected();
    }

    void startReconnect()
    {
        if (!wantedOpen_ || port_.isEmpty()) {
            return;
        }
        emit connectionLost();
        reconnectTimer_->start();
    }

    void handlePacket(const MavPacket &packet)
    {
        QVariantMap data;
        lastMessageMs_ = linkClock_.elapsed();

        if (packet.msgId == 0 && packet.payload.size() >= 9) {
            protocol_ = LinkProtocol::Mavlink;
            mspPollTimer_->stop();
            lastHeartbeatMs_ = linkClock_.elapsed();
            targetSystem_ = packet.sysId;
            targetComponent_ = packet.compId;
            const uint32_t customMode = readLe<uint32_t>(packet.payload, 0);
            data["mode"] = arduModeName(customMode);

            if (!heartbeatSeen_) {
                heartbeatSeen_ = true;
                heartbeatTimer_->stop();
                emit statusChanged("Heartbeat received. Validating baud...", "status-warning");
                heartbeatValidationTimer_->start();
            }
        } else if (packet.msgId == 1 && packet.payload.size() >= 31) {
            const int8_t battery = readLe<int8_t>(packet.payload, 30);
            if (battery >= 0) {
                data["battery"] = QString("%1%").arg(battery);
            }
        } else if (packet.msgId == 24 && packet.payload.size() >= 30) {
            const uint8_t fixType = uint8_t(packet.payload.at(28));
            if (fixType >= 2) {
                const double lat = readLe<int32_t>(packet.payload, 8) / 1e7;
                const double lon = readLe<int32_t>(packet.payload, 12) / 1e7;
                const double alt = readLe<int32_t>(packet.payload, 16) / 1000.0;
                data["gps"] = QString("%1 N, %2 E").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6);
                data["current_pos"] = QString("%1, %2, %3m").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6).arg(alt, 0, 'f', 1);
                data["lat"] = lat;
                data["lon"] = lon;
                data["alt"] = alt;
            }
        } else if (packet.msgId == 30 && packet.payload.size() >= 28) {
            const double roll = readLe<float>(packet.payload, 4) * 57.2958;
            const double pitch = readLe<float>(packet.payload, 8) * 57.2958;
            const double yaw = readLe<float>(packet.payload, 12) * 57.2958;
            data["attitude"] = QString("Roll:%1 deg, Pitch:%2 deg, Yaw:%3 deg")
                                   .arg(roll, 0, 'f', 1)
                                   .arg(pitch, 0, 'f', 1)
                                   .arg(yaw, 0, 'f', 1);
            data["roll"] = roll;
            data["pitch"] = pitch;
            data["yaw"] = yaw;
        } else if (packet.msgId == 33 && packet.payload.size() >= 28) {
            const double lat = readLe<int32_t>(packet.payload, 4) / 1e7;
            const double lon = readLe<int32_t>(packet.payload, 8) / 1e7;
            const double alt = readLe<int32_t>(packet.payload, 12) / 1000.0;
            const double vx = readLe<int16_t>(packet.payload, 20);
            const double vy = readLe<int16_t>(packet.payload, 22);
            const double vz = readLe<int16_t>(packet.payload, 24);
            const double speed = std::sqrt(vx * vx + vy * vy + vz * vz) / 100.0;
            data["gps"] = QString("%1 N, %2 E").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6);
            data["current_pos"] = QString("%1, %2, %3m").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6).arg(alt, 0, 'f', 1);
            data["speed"] = QString("%1 m/s").arg(speed, 0, 'f', 1);
            data["lat"] = lat;
            data["lon"] = lon;
            data["alt"] = alt;
        } else if (packet.msgId == 109 && packet.payload.size() >= 9) {
            data["rssi"] = QString("L:%1dB R:%2dB N:%3")
                               .arg(uint8_t(packet.payload.at(0)))
                               .arg(uint8_t(packet.payload.at(1)))
                               .arg(uint8_t(packet.payload.at(3)));
        } else if (packet.msgId == 77 && packet.payload.size() >= 3) {
            const uint16_t command = readLe<uint16_t>(packet.payload, 0);
            const uint8_t result = uint8_t(packet.payload.at(2));
            emit eventLog(QString("COMMAND_ACK command=%1 result=%2").arg(command).arg(commandResultName(result)), result == 0 ? "info" : "warn");
        } else if (packet.msgId == 253 && packet.payload.size() >= 51) {
            const uint8_t severity = uint8_t(packet.payload.at(0));
            QByteArray rawText = packet.payload.mid(1, 50);
            const int nullIndex = rawText.indexOf('\0');
            if (nullIndex >= 0) {
                rawText.truncate(nullIndex);
            }
            const QString text = QString::fromLatin1(rawText).trimmed();
            if (!text.isEmpty()) {
                emit eventLog(QString("STATUSTEXT [%1]: %2").arg(severity).arg(text), severity <= 4 ? "warn" : "info");
                if (text.startsWith("BAUD=", Qt::CaseInsensitive)) {
                    bool ok = false;
                    const int advertised = text.mid(5).trimmed().toInt(&ok);
                    if (ok) {
                        baudAdvertised_ = advertised;
                        if (baudAdvertised_ != baud_) {
                            failBaudMismatch();
                            return;
                        }
                        if (heartbeatSeen_ && !heartbeatAccepted_) {
                            acceptHeartbeatConnection();
                        }
                    }
                }
            }
        }

        if (!data.isEmpty()) {
            emit telemetryUpdate(data);
        }
    }

    void handleMspPacket(const MspPacket &packet)
    {
        if (protocol_ == LinkProtocol::Mavlink) {
            return;
        }

        lastMessageMs_ = linkClock_.elapsed();

        if (packet.error) {
            emit eventLog(QString("MSP error response for cmd=%1").arg(packet.cmd), "warn");
            return;
        }

        acceptMspConnection();

        QVariantMap data;
        switch (packet.cmd) {
        case MspApiVersion:
            if (packet.payload.size() >= 3) {
                mspApiVersion_ = QString("%1.%2").arg(uint8_t(packet.payload.at(1))).arg(uint8_t(packet.payload.at(2)));
                data["mode"] = mspIdentityText();
            }
            break;
        case MspFcVariant:
            if (packet.payload.size() >= 4) {
                mspFcVariant_ = QString::fromLatin1(packet.payload.left(4));
                data["mode"] = mspIdentityText();
            }
            break;
        case MspFcVersion:
            if (packet.payload.size() >= 3) {
                mspFcVersion_ = QString("%1.%2.%3")
                                    .arg(uint8_t(packet.payload.at(0)))
                                    .arg(uint8_t(packet.payload.at(1)))
                                    .arg(uint8_t(packet.payload.at(2)));
                data["mode"] = mspIdentityText();
            }
            break;
        case MspStatus:
            if (packet.payload.size() >= 6) {
                const uint16_t cycleTime = readLe<uint16_t>(packet.payload, 0);
                const uint16_t sensors = readLe<uint16_t>(packet.payload, 4);
                data["mode"] = QString("%1 | cycle %2us | sensors 0x%3")
                                   .arg(mspIdentityText())
                                   .arg(cycleTime)
                                   .arg(QString::number(sensors, 16).toUpper());
            }
            break;
        case MspRawGps:
            if (packet.payload.size() >= 16) {
                const uint8_t fixType = uint8_t(packet.payload.at(0));
                const uint8_t satellites = uint8_t(packet.payload.at(1));
                const int32_t latRaw = readLe<int32_t>(packet.payload, 2);
                const int32_t lonRaw = readLe<int32_t>(packet.payload, 6);
                const uint16_t altM = readLe<uint16_t>(packet.payload, 10);
                const uint16_t speedCms = readLe<uint16_t>(packet.payload, 12);
                data["speed"] = QString("%1 m/s").arg(speedCms / 100.0, 0, 'f', 1);
                if (fixType >= 2 && latRaw != 0 && lonRaw != 0) {
                    const double lat = latRaw / 1e7;
                    const double lon = lonRaw / 1e7;
                    data["gps"] = QString("%1 N, %2 E (%3 sats)").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6).arg(satellites);
                    data["current_pos"] = QString("%1, %2, %3m").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6).arg(altM);
                    data["lat"] = lat;
                    data["lon"] = lon;
                    data["alt"] = double(altM);
                } else {
                    data["gps"] = QString("No fix (%1 sats)").arg(satellites);
                }
            }
            break;
        case MspAttitude:
            if (packet.payload.size() >= 6) {
                const double roll = readLe<int16_t>(packet.payload, 0) / 10.0;
                const double pitch = readLe<int16_t>(packet.payload, 2) / 10.0;
                const double yaw = readLe<int16_t>(packet.payload, 4);
                data["attitude"] = QString("Roll:%1 deg, Pitch:%2 deg, Yaw:%3 deg")
                                       .arg(roll, 0, 'f', 1)
                                       .arg(pitch, 0, 'f', 1)
                                       .arg(yaw, 0, 'f', 1);
                data["roll"] = roll;
                data["pitch"] = pitch;
                data["yaw"] = yaw;
            }
            break;
        case MspAltitude:
            if (packet.payload.size() >= 6) {
                const double altitudeM = readLe<int32_t>(packet.payload, 0) / 100.0;
                const double varioMs = readLe<int16_t>(packet.payload, 4) / 100.0;
                data["altitude_aux"] = QString("Alt:%1 m | Vario:%2 m/s").arg(altitudeM, 0, 'f', 2).arg(varioMs, 0, 'f', 2);
            }
            break;
        case MspAnalog:
            if (packet.payload.size() >= 1) {
                const double voltage = uint8_t(packet.payload.at(0)) / 10.0;
                QString battery = QString("%1 V").arg(voltage, 0, 'f', 1);
                if (packet.payload.size() >= 7) {
                    const uint16_t rssi = readLe<uint16_t>(packet.payload, 3);
                    const int16_t currentCa = readLe<int16_t>(packet.payload, 5);
                    battery += QString(" | %1 A").arg(currentCa / 100.0, 0, 'f', 2);
                    data["altitude_aux"] = QString("RSSI:%1").arg(rssi);
                }
                data["battery"] = battery;
            }
            break;
        case MspBatteryState:
            if (packet.payload.size() >= 9) {
                const uint8_t cells = uint8_t(packet.payload.at(0));
                const double voltage = uint8_t(packet.payload.at(3)) / 10.0;
                const int16_t currentCa = readLe<int16_t>(packet.payload, 6);
                const uint8_t state = uint8_t(packet.payload.at(8));
                data["battery"] = QString("%1 V | cells %2 | %3 A | state %4")
                                      .arg(voltage, 0, 'f', 1)
                                      .arg(cells)
                                      .arg(currentCa / 100.0, 0, 'f', 2)
                                      .arg(state);
            }
            break;
        default:
            break;
        }

        if (!mspIdentityLogged_ && !mspFcVariant_.isEmpty() && !mspFcVersion_.isEmpty()) {
            mspIdentityLogged_ = true;
            emit eventLog(QString("MSP detected: %1 %2, API %3 on %4 at %5 baud.")
                              .arg(mspFcVariant_, mspFcVersion_, mspApiVersion_, port_)
                              .arg(baud_),
                          "info");
        }

        if (!data.isEmpty()) {
            emit telemetryUpdate(data);
        }
    }

    void acceptMspConnection()
    {
        if (mspAccepted_) {
            return;
        }

        protocol_ = LinkProtocol::Msp;
        mspAccepted_ = true;
        heartbeatSeen_ = false;
        heartbeatAccepted_ = false;
        reconnectAttempts_ = 0;
        heartbeatTimer_->stop();
        heartbeatValidationTimer_->stop();
        gcsHeartbeatTimer_->stop();
        emit statusChanged("Connected (MSP)", "status-connected");
        emit eventLog("MSP link accepted. Polling USB telemetry from flight controller.", "info");
        emit connected();
    }

    void pollMsp()
    {
        if (!serial_->isOpen() || protocol_ == LinkProtocol::Mavlink) {
            return;
        }

        ++mspPollTick_;
        if (!mspAccepted_) {
            static constexpr uint8_t discovery[] = {MspApiVersion, MspFcVariant, MspFcVersion, MspStatus};
            sendMspRequest(discovery[mspPollTick_ % 4]);
            return;
        }

        sendMspRequest(MspAttitude);
        if (mspPollTick_ % 3 == 0) {
            sendMspRequest(MspAltitude);
        }
        if (mspPollTick_ % 5 == 0) {
            sendMspRequest(MspRawGps);
        }
        if (mspPollTick_ % 10 == 0) {
            sendMspRequest(MspAnalog);
            sendMspRequest(MspBatteryState);
        }
        if (mspPollTick_ % 20 == 0) {
            sendMspRequest(MspStatus);
        }
    }

    void sendMspRequest(uint8_t cmd)
    {
        if (!serial_->isOpen()) {
            return;
        }

        const QByteArray packet = makeMspRequest(cmd);
        bytesSent_ += uint64_t(packet.size());
        bytesSentThisSecond_ += uint64_t(packet.size());
        serial_->write(packet);
        serial_->flush();
    }

    QString mspIdentityText() const
    {
        QStringList parts;
        if (!mspFcVariant_.isEmpty()) {
            parts << mspFcVariant_;
        }
        if (!mspFcVersion_.isEmpty()) {
            parts << mspFcVersion_;
        }
        if (!mspApiVersion_.isEmpty()) {
            parts << QString("MSP %1").arg(mspApiVersion_);
        }
        return parts.isEmpty() ? "MSP" : parts.join(" ");
    }

    void sendCommandLong(uint16_t command, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0)
    {
        ensureOpen();
        ensureMavlinkCommand();

        QByteArray payload(33, char(0));
        writeLe<float>(payload, 0, p1);
        writeLe<float>(payload, 4, p2);
        writeLe<float>(payload, 8, p3);
        writeLe<float>(payload, 12, p4);
        writeLe<float>(payload, 16, p5);
        writeLe<float>(payload, 20, p6);
        writeLe<float>(payload, 24, p7);
        writeLe<uint16_t>(payload, 28, command);
        payload[30] = char(targetSystem_);
        payload[31] = char(targetComponent_);
        payload[32] = char(0);
        writePacket(76, payload);
        emit eventLog(QString("Sent COMMAND_LONG %1").arg(command), "tx");
    }

    void setMode(uint32_t customMode)
    {
        ensureOpen();
        ensureMavlinkCommand();

        QByteArray payload(6, char(0));
        writeLe<uint32_t>(payload, 0, customMode);
        payload[4] = char(targetSystem_);
        payload[5] = char(MavModeFlagCustomModeEnabled);
        writePacket(11, payload);
        emit eventLog(QString("Sent SET_MODE custom_mode=%1").arg(customMode), "tx");
    }

    void ensureOpen() const
    {
        if (!serial_->isOpen()) {
            throw std::runtime_error("MAVLink is not connected");
        }
    }

    void ensureMavlinkCommand() const
    {
        if (protocol_ == LinkProtocol::Msp) {
            throw std::runtime_error("Flight commands are not supported on the USB MSP/Betaflight link");
        }
    }

    void writePacket(uint8_t msgId, const QByteArray &payload)
    {
        const QByteArray packet = makeV1Packet(sequence_, groundSystem_, groundComponent_, msgId, payload);
        bytesSent_ += uint64_t(packet.size());
        bytesSentThisSecond_ += uint64_t(packet.size());
        serial_->write(packet);
        serial_->flush();
    }

    void sendGcsHeartbeat()
    {
        if (!serial_->isOpen()) {
            return;
        }

        QByteArray payload(9, char(0));
        writeLe<uint32_t>(payload, 0, 0);
        payload[4] = char(MavTypeGcs);
        payload[5] = char(MavAutopilotInvalid);
        payload[6] = char(0);
        payload[7] = char(MavStateActive);
        payload[8] = char(3);
        writePacket(0, payload);
    }

    void requestTelemetryStreams()
    {
        if (!serial_->isOpen()) {
            return;
        }

        requestDataStream(MavDataStreamExtendedStatus, 2);
        requestDataStream(MavDataStreamPosition, 4);
        requestDataStream(MavDataStreamExtra1, 10);
        requestDataStream(MavDataStreamExtra2, 4);
        requestDataStream(MavDataStreamExtra3, 2);
        emit eventLog("Requested MAVLink telemetry streams from vehicle.", "tx");
    }

    void requestDataStream(uint8_t streamId, uint16_t rateHz)
    {
        QByteArray payload(6, char(0));
        writeLe<uint16_t>(payload, 0, rateHz);
        payload[2] = char(targetSystem_);
        payload[3] = char(targetComponent_);
        payload[4] = char(streamId);
        payload[5] = char(1);
        writePacket(66, payload);
    }

    QString commandResultName(uint8_t result) const
    {
        switch (result) {
        case 0: return "ACCEPTED";
        case 1: return "TEMPORARILY_REJECTED";
        case 2: return "DENIED";
        case 3: return "UNSUPPORTED";
        case 4: return "FAILED";
        case 5: return "IN_PROGRESS";
        case 6: return "CANCELLED";
        default: return QString("UNKNOWN_%1").arg(result);
        }
    }

    void publishLinkStats()
    {
        QVariantMap data;
        const qint64 now = linkClock_.elapsed();
        const qint64 lastHeartbeatAge = lastHeartbeatMs_ >= 0 ? now - lastHeartbeatMs_ : -1;
        const qint64 lastMessageAge = lastMessageMs_ >= 0 ? now - lastMessageMs_ : -1;

        QString health = "No Data";
        QString style = "status-disconnected";
        if (serial_->isOpen()) {
            if (protocol_ == LinkProtocol::Msp && lastMessageAge >= 0 && lastMessageAge <= 3000) {
                health = "Healthy (MSP)";
                style = "status-connected";
            } else if (lastHeartbeatAge >= 0 && lastHeartbeatAge <= 3000) {
                health = "Healthy";
                style = "status-connected";
            } else if (lastMessageAge >= 0 && lastMessageAge <= 7000) {
                health = "Telemetry Only";
                style = "status-warning";
            } else if (lastHeartbeatAge >= 0 && lastHeartbeatAge <= 12000) {
                health = "Weak Link";
                style = "status-warning";
            } else {
                health = "No Heartbeat";
                style = "status-disconnected";
            }
        }

        data["health"] = health;
        data["style"] = style;
        data["rx_rate"] = QString("%1 B/s").arg(bytesReceivedThisSecond_);
        data["tx_rate"] = QString("%1 B/s").arg(bytesSentThisSecond_);
        data["packet_rate"] = QString("%1 pkt/s").arg(packetsReceivedThisSecond_);
        data["total_rx"] = QString::number(bytesReceived_);
        data["total_tx"] = QString::number(bytesSent_);
        data["last_heartbeat"] = protocol_ == LinkProtocol::Msp ? "N/A (MSP)" : (lastHeartbeatAge >= 0 ? QString("%1 ms ago").arg(lastHeartbeatAge) : "Never");
        data["last_message"] = lastMessageAge >= 0 ? QString("%1 ms ago").arg(lastMessageAge) : "Never";
        emit linkStatsUpdate(data);

        bytesReceivedThisSecond_ = 0;
        bytesSentThisSecond_ = 0;
        packetsReceivedThisSecond_ = 0;
    }

    QSerialPort *serial_;
    QTimer *heartbeatTimer_;
    QTimer *heartbeatValidationTimer_;
    QTimer *gcsHeartbeatTimer_;
    QTimer *mspPollTimer_;
    QTimer *reconnectTimer_;
    QTimer *linkStatsTimer_;
    MavParser parser_;
    MspParser mspParser_;
    QElapsedTimer linkClock_;
    QString port_;
    int baud_ = 115200;
    int reconnectAttempts_ = 0;
    const int maxReconnectAttempts_ = 5;
    bool heartbeatSeen_ = false;
    bool heartbeatAccepted_ = false;
    bool mspAccepted_ = false;
    bool wantedOpen_ = true;
    LinkProtocol protocol_ = LinkProtocol::Unknown;
    int baudAdvertised_ = -1;
    int mspPollTick_ = 0;
    bool mspIdentityLogged_ = false;
    QString mspApiVersion_;
    QString mspFcVariant_;
    QString mspFcVersion_;
    uint8_t sequence_ = 0;
    uint8_t targetSystem_ = 1;
    uint8_t targetComponent_ = 1;
    const uint8_t groundSystem_ = 255;
    const uint8_t groundComponent_ = 190;
    qint64 lastHeartbeatMs_ = -1;
    qint64 lastMessageMs_ = -1;
    uint64_t bytesReceived_ = 0;
    uint64_t bytesSent_ = 0;
    uint64_t packetsReceived_ = 0;
    uint64_t bytesReceivedThisSecond_ = 0;
    uint64_t bytesSentThisSecond_ = 0;
    uint64_t packetsReceivedThisSecond_ = 0;
};

class DroneControlApp : public QWidget {
    Q_OBJECT

public:
    explicit DroneControlApp(QWidget *parent = nullptr)
        : QWidget(parent),
          worker_(new MavlinkWorker),
          workerThread_(new QThread(this))
    {
        worker_->moveToThread(workerThread_);
        connect(workerThread_, &QThread::finished, worker_, &QObject::deleteLater);
        workerThread_->start();

        setWindowTitle("AeroLink Drone Ground Station");
        setMinimumSize(1120, 720);
        setupUi();
        connectSignals();
    }

    ~DroneControlApp() override
    {
        QMetaObject::invokeMethod(worker_, [worker = worker_]() {
            worker->disconnectFromVehicle();
        }, Qt::BlockingQueuedConnection);
        workerThread_->quit();
        workerThread_->wait(3000);
    }

private:
    QLabel *makeValueLabel(const QString &text = "--")
    {
        auto *label = new QLabel(text);
        label->setProperty("class", "value");
        label->setMinimumHeight(30);
        return label;
    }

    QLabel *makeFieldLabel(const QString &text)
    {
        auto *label = new QLabel(text);
        label->setProperty("class", "fieldLabel");
        return label;
    }

    QGroupBox *makeGroup(const QString &title)
    {
        auto *group = new QGroupBox(title);
        group->setProperty("class", "panel");
        return group;
    }

    void setupUi()
    {
        setStyleSheet(R"(
QWidget {
    background: #090F1D;
    color: #E5F0FF;
    font-family: "Segoe UI", "Arial";
    font-size: 12px;
}
QFrame#topBar {
    background: #10192B;
    border: 1px solid #223451;
    border-radius: 8px;
}
QLabel#appTitle {
    color: #F5FAFF;
    font-size: 24px;
    font-weight: 800;
}
QLabel#appSubtitle {
    color: #8EA4C8;
    font-size: 12px;
}
QGroupBox {
    background: #10192B;
    border: 1px solid #243653;
    border-radius: 8px;
    margin-top: 16px;
    padding: 10px;
    font-weight: 700;
    color: #D9E7FF;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #56CCF2;
}
QLabel {
    background: transparent;
    color: #C9D8F2;
}
QLabel[class="fieldLabel"] {
    color: #93A7C7;
    font-weight: 700;
    padding: 0;
}
QLabel[class="value"] {
    background: #0B1324;
    border: 1px solid #243655;
    border-radius: 6px;
    color: #6EE7B7;
    font-weight: 700;
    padding: 6px 8px;
}
QLabel#currentPositionValue {
    min-height: 30px;
    max-height: 40px;
}
QLabel#statusPill {
    background: #24151B;
    border: 1px solid #613243;
    border-radius: 14px;
    color: #FF6B86;
    font-weight: 800;
    padding: 7px 12px;
}
QComboBox, QLineEdit, QTextEdit {
    background: #0B1324;
    border: 1px solid #314462;
    border-radius: 6px;
    color: #F4F8FF;
    padding: 7px;
    selection-background-color: #1E88E5;
}
QLineEdit, QComboBox {
    min-height: 24px;
}
QTextEdit {
    font-family: "Consolas", "Courier New";
}
QPushButton {
    border: none;
    border-radius: 5px;
    color: white;
    font-weight: 800;
    min-height: 28px;
    padding: 6px 14px;
}
QPushButton:disabled {
    background: #263246;
    color: #7887A1;
}
QPushButton#btnConnect { background: #00A3FF; }
QPushButton#btnRefreshPorts { background: #1F2A44; border: 1px solid #324A70; }
QPushButton#btnArm,
QPushButton#btnTakeoff,
QPushButton#btnLand,
QPushButton#btnRTH {
    background: #26364F;
    border: 1px solid #3A5278;
    color: #DCEAFF;
}
QPushButton#btnGoToTarget { background: #10B981; }
QPushButton#btnClearHistory { background: #475569; }
QPushButton:hover:!disabled {
    border: 1px solid #A7C7FF;
}
QTextEdit#eventLog {
    color: #B9C8E6;
}
QTabWidget::pane {
    border: 1px solid #243653;
    border-radius: 8px;
    top: -1px;
}
QTabBar::tab {
    background: #10192B;
    border: 1px solid #243653;
    color: #93A7C7;
    padding: 9px 18px;
    margin-right: 4px;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
}
QTabBar::tab:selected {
    background: #17243A;
    color: #56CCF2;
}
)");

        auto *root = new QVBoxLayout(this);
        root->setContentsMargins(18, 14, 18, 16);
        root->setSpacing(8);

        auto *topBar = new QFrame;
        topBar->setObjectName("topBar");
        auto *topLayout = new QHBoxLayout(topBar);
        topLayout->setContentsMargins(20, 12, 20, 12);

        auto *titleBlock = new QVBoxLayout;
        auto *title = new QLabel("AeroLink Drone Ground Station");
        title->setObjectName("appTitle");
        auto *subtitle = new QLabel("MAVLink/MSP command center | telemetry monitor | autonomous target navigation");
        subtitle->setObjectName("appSubtitle");
        titleBlock->addWidget(title);
        titleBlock->addWidget(subtitle);

        labelStatus_ = new QLabel("Disconnected");
        labelStatus_->setObjectName("statusPill");
        labelStatus_->setAlignment(Qt::AlignCenter);
        labelStatus_->setMinimumWidth(180);

        topLayout->addLayout(titleBlock, 1);
        topLayout->addWidget(labelStatus_);
        root->addWidget(topBar);

        auto *tabs = new QTabWidget;
        auto *controlPage = new QWidget;
        auto *controlRoot = new QVBoxLayout(controlPage);
        controlRoot->setContentsMargins(0, 10, 0, 0);
        controlRoot->setSpacing(8);

        auto *connectionGroup = makeGroup("LINK CONFIGURATION");
        auto *connectionLayout = new QGridLayout(connectionGroup);
        connectionLayout->setHorizontalSpacing(12);
        connectionLayout->setVerticalSpacing(10);

        comboCom_ = new QComboBox;
        comboCom_->setEditable(true);
        comboCom_->setMinimumWidth(300);

        comboBaud_ = new QComboBox;
        comboBaud_->addItems({"115200", "57600", "460800", "38400", "19200", "9600", "921600"});
        comboBaud_->setEditable(true);
        comboBaud_->setInsertPolicy(QComboBox::NoInsert);
        comboBaud_->setMinimumWidth(120);

        btnConnect_ = new QPushButton("Connect");
        btnConnect_->setObjectName("btnConnect");
        btnConnect_->setMinimumWidth(150);
        btnRefreshPorts_ = new QPushButton("Refresh");
        btnRefreshPorts_->setObjectName("btnRefreshPorts");
        btnRefreshPorts_->setMinimumWidth(100);

        connectionLayout->addWidget(makeFieldLabel("Serial Port"), 0, 0);
        connectionLayout->addWidget(comboCom_, 0, 1);
        connectionLayout->addWidget(makeFieldLabel("Baud Rate"), 0, 2);
        connectionLayout->addWidget(comboBaud_, 0, 3);
        connectionLayout->addWidget(btnRefreshPorts_, 0, 4);
        connectionLayout->addWidget(btnConnect_, 0, 5);
        connectionLayout->setColumnStretch(1, 2);
        connectionLayout->setColumnStretch(6, 1);
        controlRoot->addWidget(connectionGroup);

        auto *middleLayout = new QHBoxLayout;
        middleLayout->setSpacing(12);

        auto *commandGroup = makeGroup("FLIGHT COMMANDS");
        commandGroup->setMinimumHeight(170);
        auto *commandLayout = new QGridLayout(commandGroup);
        commandLayout->setContentsMargins(16, 18, 16, 16);
        commandLayout->setHorizontalSpacing(12);
        commandLayout->setVerticalSpacing(12);

        btnArm_ = new QPushButton("ARM");
        btnArm_->setObjectName("btnArm");
        btnTakeoff_ = new QPushButton("TAKE OFF");
        btnTakeoff_->setObjectName("btnTakeoff");
        btnLand_ = new QPushButton("LAND");
        btnLand_->setObjectName("btnLand");
        btnRth_ = new QPushButton("RETURN HOME");
        btnRth_->setObjectName("btnRTH");
        for (QPushButton *button : {btnArm_, btnTakeoff_, btnLand_, btnRth_}) {
            button->setMinimumHeight(44);
            button->setMaximumHeight(46);
            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        }

        commandLayout->addWidget(btnArm_, 0, 0);
        commandLayout->addWidget(btnTakeoff_, 0, 1);
        commandLayout->addWidget(btnLand_, 1, 0);
        commandLayout->addWidget(btnRth_, 1, 1);
        commandLayout->setColumnStretch(0, 1);
        commandLayout->setColumnStretch(1, 1);

        auto *targetGroup = makeGroup("AUTONOMOUS TARGET");
        targetGroup->setMinimumHeight(230);
        auto *targetLayout = new QGridLayout(targetGroup);
        targetLayout->setHorizontalSpacing(10);
        targetLayout->setVerticalSpacing(10);

        inputTargetLat_ = new QLineEdit;
        inputTargetLat_->setPlaceholderText("Latitude, e.g. 10.776");
        inputTargetLon_ = new QLineEdit;
        inputTargetLon_->setPlaceholderText("Longitude, e.g. 106.697");
        inputTargetAlt_ = new QLineEdit;
        inputTargetAlt_->setPlaceholderText("Altitude in meters");
        btnGoToTarget_ = new QPushButton("GO TO TARGET");
        btnGoToTarget_->setObjectName("btnGoToTarget");
        btnGoToTarget_->setMinimumHeight(40);
        valueCurrentPos_ = makeValueLabel("--");
        valueCurrentPos_->setObjectName("currentPositionValue");
        valueCurrentPos_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        targetLayout->addWidget(makeFieldLabel("Latitude"), 0, 0);
        targetLayout->addWidget(inputTargetLat_, 0, 1);
        targetLayout->addWidget(makeFieldLabel("Longitude"), 0, 2);
        targetLayout->addWidget(inputTargetLon_, 0, 3);
        targetLayout->addWidget(makeFieldLabel("Altitude"), 1, 0);
        targetLayout->addWidget(inputTargetAlt_, 1, 1);
        targetLayout->addWidget(btnGoToTarget_, 1, 2, 1, 2);
        targetLayout->addWidget(makeFieldLabel("Current Position"), 2, 0);
        targetLayout->addWidget(valueCurrentPos_, 2, 1, 1, 3);
        targetLayout->setColumnStretch(1, 1);
        targetLayout->setColumnStretch(3, 1);
        targetLayout->setRowStretch(3, 1);

        middleLayout->addWidget(commandGroup, 1);
        middleLayout->addWidget(targetGroup, 3);
        controlRoot->addLayout(middleLayout, 0);

        auto *linkHealthGroup = makeGroup("SERIAL LINK HEALTH");
        auto *linkHealthLayout = new QGridLayout(linkHealthGroup);
        linkHealthLayout->setHorizontalSpacing(10);
        linkHealthLayout->setVerticalSpacing(10);

        valueLinkHealth_ = makeValueLabel("No Data");
        valueRxRate_ = makeValueLabel("0 B/s");
        valueTxRate_ = makeValueLabel("0 B/s");
        valuePacketRate_ = makeValueLabel("0 pkt/s");
        valueLastHeartbeat_ = makeValueLabel("Never");
        valueLastMessage_ = makeValueLabel("Never");

        linkHealthLayout->addWidget(makeFieldLabel("Health"), 0, 0);
        linkHealthLayout->addWidget(valueLinkHealth_, 0, 1);
        linkHealthLayout->addWidget(makeFieldLabel("RX Rate"), 0, 2);
        linkHealthLayout->addWidget(valueRxRate_, 0, 3);
        linkHealthLayout->addWidget(makeFieldLabel("TX Rate"), 0, 4);
        linkHealthLayout->addWidget(valueTxRate_, 0, 5);
        linkHealthLayout->addWidget(makeFieldLabel("Packet Rate"), 1, 0);
        linkHealthLayout->addWidget(valuePacketRate_, 1, 1);
        linkHealthLayout->addWidget(makeFieldLabel("Last Heartbeat"), 1, 2);
        linkHealthLayout->addWidget(valueLastHeartbeat_, 1, 3);
        linkHealthLayout->addWidget(makeFieldLabel("Last Message"), 1, 4);
        linkHealthLayout->addWidget(valueLastMessage_, 1, 5);
        linkHealthLayout->setColumnStretch(1, 1);
        linkHealthLayout->setColumnStretch(3, 1);
        linkHealthLayout->setColumnStretch(5, 1);
        controlRoot->addWidget(linkHealthGroup);

        auto *telemetryGroup = makeGroup("LIVE TELEMETRY");
        auto *telemetryLayout = new QGridLayout(telemetryGroup);
        telemetryLayout->setHorizontalSpacing(10);
        telemetryLayout->setVerticalSpacing(10);

        valueBattery_ = makeValueLabel("--");
        valueAltitude_ = makeValueLabel("L:-- R:--");
        valueGps_ = makeValueLabel("--");
        valueAttitude_ = makeValueLabel("Roll:--, Pitch:--, Yaw:--");
        valueSpeed_ = makeValueLabel("--");
        valueFlightStatus_ = makeValueLabel("--");

        telemetryLayout->addWidget(makeFieldLabel("Battery"), 0, 0);
        telemetryLayout->addWidget(valueBattery_, 0, 1);
        telemetryLayout->addWidget(makeFieldLabel("Aux / Alt"), 0, 2);
        telemetryLayout->addWidget(valueAltitude_, 0, 3);
        telemetryLayout->addWidget(makeFieldLabel("GPS"), 1, 0);
        telemetryLayout->addWidget(valueGps_, 1, 1);
        telemetryLayout->addWidget(makeFieldLabel("Attitude"), 1, 2);
        telemetryLayout->addWidget(valueAttitude_, 1, 3);
        telemetryLayout->addWidget(makeFieldLabel("Speed"), 2, 0);
        telemetryLayout->addWidget(valueSpeed_, 2, 1);
        telemetryLayout->addWidget(makeFieldLabel("Flight Mode"), 2, 2);
        telemetryLayout->addWidget(valueFlightStatus_, 2, 3);
        telemetryLayout->setColumnStretch(1, 1);
        telemetryLayout->setColumnStretch(3, 1);
        controlRoot->addWidget(telemetryGroup);

        auto *bottomLayout = new QHBoxLayout;
        bottomLayout->setSpacing(12);

        auto *historyGroup = makeGroup("POSITION HISTORY");
        auto *historyLayout = new QVBoxLayout(historyGroup);
        textPositionHistory_ = new QTextEdit;
        textPositionHistory_->setReadOnly(true);
        textPositionHistory_->setPlaceholderText("Position stream will appear here when GPS telemetry is available.");
        textPositionHistory_->setMinimumHeight(120);
        btnClearHistory_ = new QPushButton("CLEAR HISTORY");
        btnClearHistory_->setObjectName("btnClearHistory");
        btnClearHistory_->setMinimumWidth(140);

        auto *historyButtonLayout = new QHBoxLayout;
        historyButtonLayout->addStretch(1);
        historyButtonLayout->addWidget(btnClearHistory_);
        historyLayout->addWidget(textPositionHistory_);
        historyLayout->addLayout(historyButtonLayout);

        auto *eventGroup = makeGroup("LINK EVENT LOG");
        auto *eventLayout = new QVBoxLayout(eventGroup);
        textEventLog_ = new QTextEdit;
        textEventLog_->setObjectName("eventLog");
        textEventLog_->setReadOnly(true);
        textEventLog_->setMinimumHeight(120);
        textEventLog_->setPlaceholderText("MAVLink ACK/STATUSTEXT and MSP serial events will appear here.");
        eventLayout->addWidget(textEventLog_);

        bottomLayout->addWidget(historyGroup, 3);
        bottomLayout->addWidget(eventGroup, 2);
        controlRoot->addLayout(bottomLayout, 1);

        auto *mapPage = new QWidget;
        auto *mapLayout = new QVBoxLayout(mapPage);
        mapLayout->setContentsMargins(12, 12, 12, 12);
        mapWidget_ = new DroneMapWidget;
        mapLayout->addWidget(mapWidget_, 1);

        auto *chartPage = new QWidget;
        auto *chartLayout = new QVBoxLayout(chartPage);
        chartLayout->setContentsMargins(12, 12, 12, 12);
        attitudeChart_ = new AttitudeChartWidget;
        chartLayout->addWidget(attitudeChart_, 1);

        tabs->addTab(controlPage, "Control");
        tabs->addTab(mapPage, "Map");
        tabs->addTab(chartPage, "Attitude Chart");
        root->addWidget(tabs, 1);

        refreshComPorts(false);
        setControlsEnabled(true);
    }

    void connectSignals()
    {
        connect(btnConnect_, &QPushButton::clicked, this, &DroneControlApp::onConnectClicked);
        connect(btnRefreshPorts_, &QPushButton::clicked, this, [this]() {
            refreshComPorts(true);
        });
        connect(comboBaud_, &QComboBox::textActivated, this, &DroneControlApp::onBaudRateChanged);
        if (comboBaud_->lineEdit()) {
            connect(comboBaud_->lineEdit(), &QLineEdit::editingFinished, this, [this]() {
                onBaudRateChanged(comboBaud_->currentText());
            });
        }
        connect(btnArm_, &QPushButton::clicked, this, &DroneControlApp::onArmClicked);
        connect(btnTakeoff_, &QPushButton::clicked, this, &DroneControlApp::onTakeoffClicked);
        connect(btnLand_, &QPushButton::clicked, this, &DroneControlApp::onLandClicked);
        connect(btnRth_, &QPushButton::clicked, this, &DroneControlApp::onRthClicked);
        connect(btnGoToTarget_, &QPushButton::clicked, this, &DroneControlApp::onGotoClicked);
        connect(btnClearHistory_, &QPushButton::clicked, textPositionHistory_, &QTextEdit::clear);

        connect(worker_, &MavlinkWorker::statusChanged, this, &DroneControlApp::onStatusChanged);
        connect(worker_, &MavlinkWorker::telemetryUpdate, this, &DroneControlApp::onTelemetryUpdate);
        connect(worker_, &MavlinkWorker::linkStatsUpdate, this, &DroneControlApp::onLinkStatsUpdate);
        connect(worker_, &MavlinkWorker::eventLog, this, &DroneControlApp::onEventLog);
        connect(worker_, &MavlinkWorker::connected, this, &DroneControlApp::onConnected);
        connect(worker_, &MavlinkWorker::disconnected, this, &DroneControlApp::onDisconnected);
        connect(worker_, &MavlinkWorker::connectionLost, this, &DroneControlApp::onConnectionLost);
    }

    void setControlsEnabled(bool enabled)
    {
        btnArm_->setEnabled(enabled);
        btnTakeoff_->setEnabled(enabled);
        btnLand_->setEnabled(enabled);
        btnRth_->setEnabled(enabled);
        btnGoToTarget_->setEnabled(enabled);
    }

private slots:
    void refreshComPorts(bool showStatus = true)
    {
        const QString current = comboCom_->currentText().trimmed();
        comboCom_->blockSignals(true);
        comboCom_->clear();

        const QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
        if (ports.isEmpty()) {
            comboCom_->addItem("No COM ports found");
        } else {
            comboCom_->addItem("Select COM Port");
            for (const QSerialPortInfo &port : ports) {
                QString label = port.portName();
                QStringList details;
                if (!port.description().isEmpty()) {
                    details << port.description();
                }
                if (!port.manufacturer().isEmpty()) {
                    details << port.manufacturer();
                }
                if (!details.isEmpty()) {
                    label += QString(" - %1").arg(details.join(" | "));
                }
                comboCom_->addItem(label, port.portName());
            }
        }

        const int previousIndex = comboCom_->findText(current, Qt::MatchStartsWith);
        if (previousIndex >= 0) {
            comboCom_->setCurrentIndex(previousIndex);
        }
        comboCom_->blockSignals(false);

        if (showStatus) {
            onStatusChanged(QString("COM ports refreshed: %1 found").arg(ports.size()), ports.isEmpty() ? "status-warning" : "status-connected");
            onEventLog(QString("Refreshed COM ports. Found %1 port(s).").arg(ports.size()), "info");
        }
    }

    void onConnectClicked()
    {
        if (connected_ || connectInProgress_) {
            connectInProgress_ = false;
            QMetaObject::invokeMethod(worker_, [worker = worker_]() {
                worker->disconnectFromVehicle();
            }, Qt::QueuedConnection);
            return;
        }

        QString port = comboCom_->currentData().toString().trimmed();
        if (port.isEmpty()) {
            port = comboCom_->currentText().trimmed();
            if (port.contains(" - ")) {
                port = port.left(port.indexOf(" - ")).trimmed();
            }
        }

        if (port.compare("Select COM Port", Qt::CaseInsensitive) == 0
            || port.compare("No COM ports found", Qt::CaseInsensitive) == 0
            || port.isEmpty()) {
            onStatusChanged("Select a COM port", "status-warning");
            return;
        }

        if (port.toLower().startsWith("com")) {
            port = QString("COM%1").arg(port.mid(3));
        }

        bool ok = false;
        int baud = comboBaud_->currentText().trimmed().toInt(&ok);
        if (!ok) {
            baud = 57600;
        }
        connectInProgress_ = true;
        btnConnect_->setText("Cancel");
        onStatusChanged("Opening serial link...", "status-warning");
        QMetaObject::invokeMethod(worker_, [worker = worker_, port, baud]() {
            worker->connectToVehicle(port, baud);
        }, Qt::QueuedConnection);
    }

    void onBaudRateChanged(const QString &text)
    {
        bool ok = false;
        const int baud = text.trimmed().toInt(&ok);
        if (!ok || baud <= 0) {
            onStatusChanged("Invalid baud rate", "status-warning");
            onEventLog(QString("Invalid baud rate input: %1").arg(text), "warn");
            return;
        }

        QMetaObject::invokeMethod(worker_, [worker = worker_, baud]() {
            worker->changeBaudRate(baud);
        }, Qt::QueuedConnection);
    }

    void onConnected()
    {
        connected_ = true;
        connectInProgress_ = false;
        btnConnect_->setText("Disconnect");
        setControlsEnabled(true);
    }

    void onDisconnected()
    {
        connected_ = false;
        connectInProgress_ = false;
        btnConnect_->setText("Connect");
        setControlsEnabled(true);
    }

    void onConnectionLost()
    {
        onStatusChanged("Radio link lost. Reconnecting...", "status-warning");
    }

    void onStatusChanged(const QString &text, const QString &style)
    {
        labelStatus_->setText(text);
        if (style == "status-connected") {
            labelStatus_->setStyleSheet("QLabel#statusPill{ background: #10261E; border: 1px solid #1C7C54; border-radius: 14px; color: #6EE7B7; font-weight: 800; padding: 7px 12px; }");
        } else if (style == "status-warning") {
            labelStatus_->setStyleSheet("QLabel#statusPill{ background: #2A2110; border: 1px solid #8A6113; border-radius: 14px; color: #FBBF24; font-weight: 800; padding: 7px 12px; }");
        } else {
            labelStatus_->setStyleSheet("QLabel#statusPill{ background: #24151B; border: 1px solid #613243; border-radius: 14px; color: #FF6B86; font-weight: 800; padding: 7px 12px; }");
        }
    }

    void onTelemetryUpdate(const QVariantMap &data)
    {
        if (data.contains("battery")) valueBattery_->setText(data["battery"].toString());
        if (data.contains("gps")) valueGps_->setText(data["gps"].toString());
        if (data.contains("attitude")) valueAttitude_->setText(data["attitude"].toString());
        if (data.contains("speed")) valueSpeed_->setText(data["speed"].toString());
        if (data.contains("mode")) valueFlightStatus_->setText(data["mode"].toString());
        if (data.contains("rssi")) valueAltitude_->setText(data["rssi"].toString());
        if (data.contains("altitude_aux")) valueAltitude_->setText(data["altitude_aux"].toString());

        if (data.contains("lat") && data.contains("lon") && data.contains("alt") && mapWidget_) {
            mapWidget_->updatePosition(data["lat"].toDouble(), data["lon"].toDouble(), data["alt"].toDouble());
        }

        if (data.contains("roll") && data.contains("pitch") && data.contains("yaw") && attitudeChart_) {
            attitudeChart_->addSample(data["roll"].toDouble(), data["pitch"].toDouble(), data["yaw"].toDouble());
        }

        if (data.contains("current_pos")) {
            const QString position = data["current_pos"].toString();
            valueCurrentPos_->setText(position);
            const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
            textPositionHistory_->append(QString("%1 (%2)").arg(position, ts));
        }
    }

    void onLinkStatsUpdate(const QVariantMap &data)
    {
        if (data.contains("health")) valueLinkHealth_->setText(data["health"].toString());
        if (data.contains("rx_rate")) valueRxRate_->setText(data["rx_rate"].toString());
        if (data.contains("tx_rate")) valueTxRate_->setText(data["tx_rate"].toString());
        if (data.contains("packet_rate")) valuePacketRate_->setText(data["packet_rate"].toString());
        if (data.contains("last_heartbeat")) valueLastHeartbeat_->setText(data["last_heartbeat"].toString());
        if (data.contains("last_message")) valueLastMessage_->setText(data["last_message"].toString());
    }

    void onEventLog(const QString &text, const QString &level)
    {
        const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
        QString prefix = "INFO";
        if (level == "warn") {
            prefix = "WARN";
        } else if (level == "tx") {
            prefix = "TX";
        }
        textEventLog_->append(QString("[%1] %2  %3").arg(ts, prefix, text));
    }

    void onArmClicked()
    {
        QMetaObject::invokeMethod(worker_, [worker = worker_]() {
            worker->arm();
        }, Qt::QueuedConnection);
    }

    void onTakeoffClicked()
    {
        bool ok = false;
        const double alt = inputTargetAlt_->text().toDouble(&ok);
        QMetaObject::invokeMethod(worker_, [worker = worker_, alt = ok ? alt : 5.0]() {
            worker->takeoff(alt);
        }, Qt::QueuedConnection);
    }

    void onLandClicked()
    {
        QMetaObject::invokeMethod(worker_, [worker = worker_]() {
            worker->land();
        }, Qt::QueuedConnection);
    }

    void onRthClicked()
    {
        QMetaObject::invokeMethod(worker_, [worker = worker_]() {
            worker->rth();
        }, Qt::QueuedConnection);
    }

    void onGotoClicked()
    {
        bool latOk = false;
        bool lonOk = false;
        bool altOk = false;
        const double lat = inputTargetLat_->text().toDouble(&latOk);
        const double lon = inputTargetLon_->text().toDouble(&lonOk);
        const double alt = inputTargetAlt_->text().toDouble(&altOk);
        if (!latOk || !lonOk || !altOk) {
            onStatusChanged("Enter a valid latitude, longitude and altitude", "status-warning");
            return;
        }

        QMetaObject::invokeMethod(worker_, [worker = worker_, lat, lon, alt]() {
            worker->goTo(lat, lon, alt);
        }, Qt::QueuedConnection);
    }

private:
    MavlinkWorker *worker_ = nullptr;
    QThread *workerThread_ = nullptr;
    bool connected_ = false;
    bool connectInProgress_ = false;
    QComboBox *comboCom_ = nullptr;
    QComboBox *comboBaud_ = nullptr;
    QPushButton *btnConnect_ = nullptr;
    QPushButton *btnRefreshPorts_ = nullptr;
    QPushButton *btnArm_ = nullptr;
    QPushButton *btnTakeoff_ = nullptr;
    QPushButton *btnLand_ = nullptr;
    QPushButton *btnRth_ = nullptr;
    QPushButton *btnGoToTarget_ = nullptr;
    QPushButton *btnClearHistory_ = nullptr;
    QLabel *labelStatus_ = nullptr;
    QLabel *valueBattery_ = nullptr;
    QLabel *valueGps_ = nullptr;
    QLabel *valueAttitude_ = nullptr;
    QLabel *valueCurrentPos_ = nullptr;
    QLabel *valueSpeed_ = nullptr;
    QLabel *valueFlightStatus_ = nullptr;
    QLabel *valueAltitude_ = nullptr;
    QLabel *valueLinkHealth_ = nullptr;
    QLabel *valueRxRate_ = nullptr;
    QLabel *valueTxRate_ = nullptr;
    QLabel *valuePacketRate_ = nullptr;
    QLabel *valueLastHeartbeat_ = nullptr;
    QLabel *valueLastMessage_ = nullptr;
    QLineEdit *inputTargetLat_ = nullptr;
    QLineEdit *inputTargetLon_ = nullptr;
    QLineEdit *inputTargetAlt_ = nullptr;
    QTextEdit *textPositionHistory_ = nullptr;
    QTextEdit *textEventLog_ = nullptr;
    DroneMapWidget *mapWidget_ = nullptr;
    AttitudeChartWidget *attitudeChart_ = nullptr;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    try {
        DroneControlApp window;
        window.show();
        return app.exec();
    } catch (const std::exception &e) {
        qCritical("Startup error: %s", e.what());
        return 1;
    }
}

#include "main.moc"
