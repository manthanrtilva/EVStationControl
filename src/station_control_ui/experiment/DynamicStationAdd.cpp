

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
    add(m_oVScrollBar);
    m_oVScrollBar.add(m_oVBox);
    // m_oVBox.pack_start(m_oSingleRow, true,true);
    // sigc::slot<bool> my_slot = sigc::mem_fun(*this, &MainWindow::OnTimeout);
    Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::OnTimeout), 2000);
    show_all();
  }
  ~MainWindow() {}

protected:
  bool OnTimeout() {
    std::cout << "Calling timeout" << std::endl;
    static int count = 0;
    SingleRow *p_SingleRow = new SingleRow();
    m_oVBox.pack_start(*p_SingleRow, true, true);
    p_SingleRow->show();
    m_oVBox.show();
    m_oCntRows[std::to_string(count)] = p_SingleRow;
    count++;
    return true;
  }

private:
  Gtk::ScrolledWindow m_oVScrollBar;
  Gtk::VBox m_oVBox;
  std::map<std::string, SingleRow *> m_oCntRows;
  // SingleRow m_oSingleRow;
};

int main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

  MainWindow win;

  // Shows the window and returns when it is closed.
  return app->run(win);
  return 0;
}