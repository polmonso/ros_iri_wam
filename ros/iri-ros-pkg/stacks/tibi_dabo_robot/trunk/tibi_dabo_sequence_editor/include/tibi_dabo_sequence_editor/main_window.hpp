/**
 * @file /include/tibi_dabo_sequence_editor/main_window.hpp
 *
 * @brief Qt based gui for tibi_dabo_sequence_editor.
 *
 * @date November 2010
 **/
#ifndef tibi_dabo_sequence_editor_MAIN_WINDOW_H
#define tibi_dabo_sequence_editor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#ifdef _HAVE_XSD
  #include "xml/motor_cfg_file.hxx"
  #include "xml/motion_sequence_file.hxx"
#endif

#include "eventserver.h"
#include "motion_sequence.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

#define NUM_JOINTS 6
namespace tibi_dabo_sequence_editor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow 
{
  Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

  public slots:
    /******************************************
     ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_load_sequence_clicked();
    void on_save_sequence_clicked();
    void on_create_sequence_clicked();
    void on_execute_sequence_clicked();
    void on_get_current_clicked();
    void on_add_step_clocked();
    void on_add_step_clocked();

  protected:
    void timerEvent(QTimerEvent * event);
    void load_joints_config(void);
    void config_joint(int index,bool enable,float min_pos,float max_pos,float min_vel, float max_vel);
    void fill_motion_table(void);

  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    // widget arrays
    QLabel *min_pos_text[NUM_JOINTS];
    QLabel *max_pos_text[NUM_JOINTS];
    QLabel *min_vel_text[NUM_JOINTS];
    QLabel *max_vel_text[NUM_JOINTS];
    QLabel *position_text[NUM_JOINTS];
    QLabel *velocity_text[NUM_JOINTS];
    QSlider *desired_pos[NUM_JOINTS];
    QSlider *desired_vel[NUM_JOINTS];
    QLineEdit *desired_pos_value[NUM_JOINTS];
    QLineEdit *desired_vel_value[NUM_JOINTS];
    // internal attributes
    std::string motion_file_name;
    std::vector<TMotionStep> seq;
    CEventServer *event_server;    
    std::vector<float> min_pos;
    std::vector<float> max_pos;
    std::vector<float> min_vel;
    std::vector<float> max_vel;
    int dof;
};

}  // namespace tibi_dabo_sequence_editor

#endif // tibi_dabo_sequence_editor_MAIN_WINDOW_H
