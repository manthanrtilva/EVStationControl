
#include <iostream>

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
    add(m_oHBox);
    m_oHBox.pack_start(m_oSingleRow, true,true);
    show_all();
  }
  ~MainWindow() {}

private:
  Gtk::HBox m_oHBox;
  SingleRow m_oSingleRow;
};

int main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

  MainWindow win;

  // Shows the window and returns when it is closed.
  return app->run(win);
  return 0;
}