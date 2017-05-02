#include <iostream>
#include <map>

#include <gtkmm.h>
class SingleRow : public Gtk::Grid {
public:
  SingleRow()
      : m_oControlButton{"unknown"}, m_oEVStatusLabel{"unknown1"}, m_oEVFaultLabel{"unknown2"},
        m_oEVkWhLabel{"unknown3"} {
    attach(m_oControlButton, 0, 0, 1, 3);
    m_oControlButton.set_halign(Gtk::Align::ALIGN_CENTER);
    m_oControlButton.set_valign(Gtk::Align::ALIGN_CENTER);
    attach(m_oEVStatusLabel, 1, 0, 1, 1);
    attach(m_oEVFaultLabel, 1, 1, 1, 1);
    attach(m_oEVkWhLabel, 1, 2, 1, 1);
    show_all();
  }
  ~SingleRow() {}

private:
  Gtk::Button m_oControlButton;
  Gtk::Label m_oEVStatusLabel;
  Gtk::Label m_oEVFaultLabel;
  Gtk::Label m_oEVkWhLabel;
};

class MainWindow : public Gtk::Window {
public:
  MainWindow() {
    m_oLandImage.set(Gdk::PixbufAnimation::create_from_file("wait_please.gif"));
    add(m_oLandImage);
    conn = Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::OnTimeout), 5000);
    show_all();
  }
  ~MainWindow() {}

protected:
  bool OnTimeout() {
    std::cout << "Calling timeout" << std::endl;
    m_pRow = new SingleRow();
    remove();
    add(*m_pRow);
    conn.disconnect();
    const Gtk::Widget* p_tempWidg = get_child();
    if(p_tempWidg)
    resize(p_tempWidg->get_width(),p_tempWidg->get_height());
    return true;
  }
private:
  Gtk::Image m_oLandImage;
  SingleRow *m_pRow{nullptr};
  sigc::connection conn;
};

int main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");
  MainWindow win;
  return app->run(win);
  return 0;
}