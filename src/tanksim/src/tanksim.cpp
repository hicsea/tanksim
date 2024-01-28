#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "tanksim/tank_frame.hpp"

class TankApp : public QApplication
{
public:
  std::shared_ptr<rclcpp::Node> nh_;

  TankApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("tanksim");
  }

  int exec()
  {
    tanksim::TankFrame *frame = new tanksim::TankFrame(nh_);
    frame->show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  TankApp app(argc, argv);
  return app.exec();
}
