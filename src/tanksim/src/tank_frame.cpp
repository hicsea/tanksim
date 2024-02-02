#include "tanksim/tank_frame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace tanksim
{

TankFrame::TankFrame(std::shared_ptr<rclcpp::Node> nh, QWidget* parent, Qt::WindowFlags f)
: nh_(nh)
, QFrame(parent, f)
, path_image_(500, 500, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("TankSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  // connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
  connect(update_timer_, &QTimer::timeout, this, &TankFrame::onUpdate);

//   nh_->setParam("background_r", DEFAULT_BG_R);
//   nh_->setParam("background_g", DEFAULT_BG_G);
//   nh_->setParam("background_b", DEFAULT_BG_B);

  QVector<QString> tanks;
  tanks.append("tank.png");


  QString images_path = "";

  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != NULL) {
    std::string path = std::string(cwd) + "/src/tanksim/images/"; // Correctly append the path
    images_path = QString::fromStdString(path); // Convert std::string to QString
  } else {
    std::cerr << "Error getting current directory" << std::endl;
  }

  for (int i = 0; i < tanks.size(); ++i)
  {
    QImage img;
    QString image_path = images_path + tanks[i];
    bool success = img.load(image_path);
    tank_images_.append(img);
  }

  meter_ = tank_images_[0].height();




  clear();

//   nh_->create_service<tanksim::srv::Empty>("clear", TankFrame::clearCallback);
//   nh_->create_service<tanksim::srv::Empty>("reset", TankFrame::resetCallback);
//   nh_->create_service<tanksim::srv::Spawn>("spawn", TankFrame::spawnCallback);
//   nh_->create_service<tanksim::srv::Kill>("kill", TankFrame::killCallback);

//  ROS_INFO("Starting tanksim with node name %s", ros::this_node::getName().c_str()) ;

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnTank("tank", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0, 0);


}


TankFrame::~TankFrame()
{
  delete update_timer_;
}



// bool TankFrame::spawnCallback(std::shared_ptr<tanksim::srv::Spawn::Request> req, 
//                                 std::shared_ptr<tanksim::srv::Spawn::Response> res)
// {
//   std::string name = spawnTank(req->name, req->x, req->y, req->theta);
//   if (name.empty())
//   {
//     // ROS_ERROR("A tankd named [%s] already exists", req.name.c_str());
//     return false;
//   }

//   res->name = name;

//   return true;
// }

// bool TankFrame::killCallback(tanksim::srv::Kill::Request& req, 
//                                tanksim::srv::Kill::Response& res)
// {
//   M_Tank::iterator it = tanks_.find(req.name);
//   if (it == tanks_.end())
//   {
//     // ROS_ERROR("Tried to kill tank [%s], which does not exist", req.name.c_str());
//     return false;
//   }

//   tanks_.erase(it);
//   update();

//   return true;
// }

// bool TankFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   ROS_INFO("Clearing tanksim.");
//   clear();
//   return true;
// }

// bool TankFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
// {
//   ROS_INFO("Resetting tanksim.");
//   tanks_.clear();
//   id_counter_ = 0;
//   spawnTank("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
//   clear();
//   return true;
// }


bool TankFrame::hasTank(const std::string& name)
{
  return tanks_.find(name) != tanks_.end();
}

// std::string TankFrame::spawnTank(const std::string& name, float x, float y, float angle)
// {
//   return spawnTank(name, x, y, angle, rand() % tank_images_.size());
// }

std::string TankFrame::spawnTank(const std::string& name, float x, float y, float angle, size_t index)
{
  std::cout << name << " spawned at (" << x << "," << y << ")" << std::endl;
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "tank" << ++id_counter_;
      real_name = ss.str();
    } while (hasTank(real_name));
  }
  else
  {
    if (hasTank(real_name))
    {
      return "";
    }
  }

  TankPtr t(new Tank(nh_, real_name, tank_images_[index], QPointF(x, height_in_meters_ - y), angle));
  
  tanks_[real_name] = t;


  update();
//    ROS_INFO("Spawning tank [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void TankFrame::clear()
{
  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;

//   nh_.param("background_r", r, r);
//   nh_.param("background_g", g, g);
//   nh_.param("background_b", b, b);

  path_image_.fill(qRgb(r, g, b));
  update();
}

void TankFrame::onUpdate()
{
  rclcpp::spin_some(nh_);

  updateTanks();

  if (!rclcpp::ok())
  {
    close();
  }
}

void TankFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Tank::iterator it = tanks_.begin();
  M_Tank::iterator end = tanks_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void TankFrame::updateTanks()
{
//   if (last_tank_update_.isZero())
//   {
//     last_tank_update_ = ros::WallTime::now();
//     return;
//   }

  bool modified = false;
  M_Tank::iterator it = tanks_.begin();
  M_Tank::iterator end = tanks_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


}