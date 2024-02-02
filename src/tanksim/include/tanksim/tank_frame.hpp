#ifndef TANK_FRAME_H
#define TANK_FRAME_H

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
#endif

# include <tanksim/srv/empty.hpp>
# include <tanksim/srv/spawn.hpp>
# include <tanksim/srv/kill.hpp>
# include <map>

# include "tank.hpp"

namespace tanksim
{

class TankFrame : public QFrame
{
  Q_OBJECT
public:
  TankFrame(std::shared_ptr<rclcpp::Node> nh, QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~TankFrame();

  std::string spawnTank(const std::string& name, float x, float y, float angle);
  std::string spawnTank(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTanks();
  void clear();
  bool hasTank(const std::string& name);

//   bool clearCallback(std::shared_ptr<tanksim::srv::Empty::Request>,
//                             std::shared_ptr<tanksim::srv::Empty::Response>);
//   bool resetCallback(std::shared_ptr<tanksim::srv::Empty::Request>, 
//                             std::shared_ptr<tanksim::srv::Empty::Response>);
//   bool spawnCallback(std::shared_ptr<tanksim::srv::Spawn::Request>, 
//                             std::shared_ptr<tanksim::srv::Spawn::Response>);
//   bool killCallback(std::shared_ptr<tanksim::srv::Kill::Request>, 
//                            std::shared_ptr<tanksim::srv::Kill::Response>);

  std::shared_ptr<rclcpp::Node> nh_;
  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  // ros::WallTime last_tank_update_;

  // ros::ServiceServer clear_srv_;
  // ros::ServiceServer reset_srv_;
  // ros::ServiceServer spawn_srv_;
  // ros::ServiceServer kill_srv_;

  typedef std::map<std::string, TankPtr> M_Tank;
  M_Tank tanks_;
  uint32_t id_counter_ = 0;

  QVector<QImage> tank_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}


#endif