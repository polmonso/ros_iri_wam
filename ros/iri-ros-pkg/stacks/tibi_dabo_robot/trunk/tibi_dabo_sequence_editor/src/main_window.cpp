/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/tibi_dabo_sequence_editor/main_window.hpp"

#include "eventexceptions.h"
#include "motor_control_exceptions.h"
#include "motor_config.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tibi_dabo_sequence_editor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
  QStringList headers;
  QString joint;
  int i=0;

  // initialize internal attributes
  this->event_server=CEventServer::instance();
  this->motion_file_name="";
  this->seq.clear();

  // assign slots to signals
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  // initialize the widget arrays for easy access
  this->min_pos_text[0]=this->ui.min_pos_text_1;
  this->min_pos_text[1]=this->ui.min_pos_text_2;
  this->min_pos_text[2]=this->ui.min_pos_text_5;
  this->min_pos_text[3]=this->ui.min_pos_text_6;
  this->min_pos_text[4]=this->ui.min_pos_text_7;
  this->min_pos_text[5]=this->ui.min_pos_text_8;
  this->max_pos_text[0]=this->ui.max_pos_text_1;
  this->max_pos_text[1]=this->ui.max_pos_text_2;
  this->max_pos_text[2]=this->ui.max_pos_text_5;
  this->max_pos_text[3]=this->ui.max_pos_text_6;
  this->max_pos_text[4]=this->ui.max_pos_text_7;
  this->max_pos_text[5]=this->ui.max_pos_text_8;
  this->min_vel_text[0]=this->ui.min_vel_text_1;
  this->min_vel_text[1]=this->ui.min_vel_text_2;
  this->min_vel_text[2]=this->ui.min_vel_text_5;
  this->min_vel_text[3]=this->ui.min_vel_text_6;
  this->min_vel_text[4]=this->ui.min_vel_text_7;
  this->min_vel_text[5]=this->ui.min_vel_text_8;
  this->max_vel_text[0]=this->ui.max_vel_text_1;
  this->max_vel_text[1]=this->ui.max_vel_text_2;
  this->max_vel_text[2]=this->ui.max_vel_text_5;
  this->max_vel_text[3]=this->ui.max_vel_text_6;
  this->max_vel_text[4]=this->ui.max_vel_text_7;
  this->max_vel_text[5]=this->ui.max_vel_text_8;
  this->desired_pos[0]=this->ui.desired_pos_1;
  this->desired_pos[1]=this->ui.desired_pos_2;
  this->desired_pos[2]=this->ui.desired_pos_5;
  this->desired_pos[3]=this->ui.desired_pos_6;
  this->desired_pos[4]=this->ui.desired_pos_7;
  this->desired_pos[5]=this->ui.desired_pos_8;
  this->desired_vel[0]=this->ui.desired_vel_1;
  this->desired_vel[1]=this->ui.desired_vel_2;
  this->desired_vel[2]=this->ui.desired_vel_5;
  this->desired_vel[3]=this->ui.desired_vel_6;
  this->desired_vel[4]=this->ui.desired_vel_7;
  this->desired_vel[5]=this->ui.desired_vel_8;
  this->desired_pos_value[0]=this->ui.desired_pos_value_1;
  this->desired_pos_value[1]=this->ui.desired_pos_value_2;
  this->desired_pos_value[2]=this->ui.desired_pos_value_5;
  this->desired_pos_value[3]=this->ui.desired_pos_value_6;
  this->desired_pos_value[4]=this->ui.desired_pos_value_7;
  this->desired_pos_value[5]=this->ui.desired_pos_value_8;
  this->desired_vel_value[0]=this->ui.desired_vel_value_1;
  this->desired_vel_value[1]=this->ui.desired_vel_value_2;
  this->desired_vel_value[2]=this->ui.desired_vel_value_5;
  this->desired_vel_value[3]=this->ui.desired_vel_value_6;
  this->desired_vel_value[4]=this->ui.desired_vel_value_7;
  this->desired_vel_value[5]=this->ui.desired_vel_value_8;
  this->position_text[0]=this->ui.position_text_1;
  this->position_text[1]=this->ui.position_text_2;
  this->position_text[2]=this->ui.position_text_3;
  this->position_text[3]=this->ui.position_text_4;
  this->position_text[4]=this->ui.position_text_5;
  this->position_text[5]=this->ui.position_text_6;
  this->velocity_text[0]=this->ui.velocity_text_1;
  this->velocity_text[1]=this->ui.velocity_text_2;
  this->velocity_text[2]=this->ui.velocity_text_3;
  this->velocity_text[3]=this->ui.velocity_text_4;
  this->velocity_text[4]=this->ui.velocity_text_5;
  this->velocity_text[5]=this->ui.velocity_text_6;

  this->load_joints_config();

  // initialize the screen widgets
  for(i=0;i<this->dof;i++)
    config_joint(i,true,this->min_pos[0],this->max_pos[0],this->min_vel[0],this->max_vel[0]);

  this->ui.motion_steps->setSortingEnabled(false);
  this->ui.motion_steps->setColumnCount(2+this->dof*2);
  headers.push_back("step no.");
  for(i=0;i<this->dof;i++) 
  {
    joint.setNum(i);
    headers.push_back("joint " + joint + " pos");
    headers.push_back("joint " + joint + " vel");
  }
  headers.push_back("delay");
  this->ui.motion_steps->setHorizontalHeaderLabels(headers);
  this->ui.motion_steps->setSelectionBehavior(QAbstractItemView::SelectRows);// only select rows
  this->ui.motion_steps->setSelectionMode(QAbstractItemView::SingleSelection);// select only one item

  // start the timer
  this->startTimer(100);
}

MainWindow::~MainWindow()
{
  this->seq.clear();
}

void MainWindow::timerEvent(QTimerEvent * event)
{
  // do periodic updates
  if(this->event_server->event_is_set(this->qnode.get_action_feedback_event_id()))
  {
    this->ui.get_current->click();
    this->event_server->reset_event(this->qnode.get_action_feedback_event_id());
  }
}

void MainWindow::config_joint(int index,bool enable,float min_pos,float max_pos,float min_vel, float max_vel)
{
  QString value;

  this->position_text[index]->setEnabled(enable);
  this->velocity_text[index]->setEnabled(enable);
  this->desired_pos[index]->setEnabled(enable);
  this->desired_vel[index]->setEnabled(enable);
  this->desired_pos_value[index]->setEnabled(enable);
  this->desired_vel_value[index]->setEnabled(enable);
  this->min_pos_text[index]->setEnabled(enable);
  value.setNum(min_pos);
  this->min_pos_text[index]->setText(value);
  this->max_pos_text[index]->setEnabled(enable);
  value.setNum(max_pos);
  this->max_pos_text[index]->setText(value);
  this->min_vel_text[index]->setEnabled(enable);
  value.setNum(min_vel);
  this->min_vel_text[index]->setText(value);
  this->max_vel_text[index]->setEnabled(enable);
  value.setNum(max_vel);
  this->max_vel_text[index]->setText(value);
  this->desired_pos[index]->setRange(min_pos,max_pos);
  this->desired_vel[index]->setRange(min_vel,max_vel);
}

void MainWindow::load_joints_config(void)
{
  motor_config_t::axis_config_iterator iterator;
  std::vector<std::string> config_files;
  std::string config_file_full_path;
  unsigned int i=0;

  config_files=this->qnode.get_config_files();
  this->dof=0;
  try{
    for(i=0;i<config_files.size();i++)
    {
      std::auto_ptr<motor_config_t> cfg(motor_config(config_files[i].c_str(),xml_schema::flags::dont_validate));
      this->dof+=cfg->num_axis();
      for(iterator=cfg->axis_config().begin();iterator!=cfg->axis_config().end();iterator++)
      {
        this->min_vel.push_back(iterator->velocity_range().min());
        this->max_vel.push_back(iterator->velocity_range().max());
        this->min_pos.push_back(iterator->position_range().min());
        this->max_pos.push_back(iterator->position_range().max());
      }
    }
  }catch (const xml_schema::exception& e){
    /* handle exceptions */
    std::ostringstream os;
    os << e;
    QMessageBox msgBox;
    msgBox.setText(QString(os.str().c_str()));
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.exec();
  }

  // wait_for the first feedback values
}

void MainWindow::fill_motion_table(void)
{
  unsigned int i=0,j=0,num_row;
  QTableWidgetItem *item;
  QString value;

  // clear all rows of the current table, if any
  num_row=this->ui.motion_steps->rowCount();
  for(i=0;i<num_row;i++)
    this->ui.motion_steps->removeRow(0);
  this->ui.motion_steps->setRowCount(this->seq.size());
  for(i=0;i<this->seq.size();i++)
  {
    value.setNum(i);
    item=new QTableWidgetItem(value);
    this->ui.motion_steps->setItem(i,0,item);
    value.setNum(this->seq[i].delay);
    item=new QTableWidgetItem(value);
    this->ui.motion_steps->setItem(i,2*this->dof+1,item);
    for(j=0;j<this->seq[i].position.size();j++)
    {
      value.setNum(this->seq[i].position[j]);
      item=new QTableWidgetItem(value);
      this->ui.motion_steps->setItem(i,2*j+1,item);
      value.setNum(this->seq[i].velocity[j]);
      item=new QTableWidgetItem(value);
      this->ui.motion_steps->setItem(i,2*j+2,item);
    }
  }
  value.setNum(this->seq.size());
  this->ui.num_steps->setText(value);
  this->ui.motion_steps->selectRow(0);
}
/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_get_current_clicked()
{
  std::list<std::string> events;
  std::vector<float> current_pos;
  std::vector<float> current_vel;
  unsigned int i=0;
  QString value;

  events.push_back(this->qnode.get_new_feedback_event_id());
  try{
    this->event_server->wait_all(events,1000);
    this->qnode.get_motion_feedback(current_pos,current_vel);
    for(i=0;i<current_pos.size();i++)
    {
      this->desired_pos[i]->setValue(current_pos[i]*180.0/3.14159);
      value.setNum(current_pos[i]*180.0/3.14159);
      this->desired_pos_value[i]->setText(value);
      this->desired_vel[i]->setValue(current_vel[i]*180.0/3.14159);
      value.setNum(current_vel[i]*180.0/3.14159);
      this->desired_vel_value[i]->setText(value);
    }
    this->event_server->reset_event(this->qnode.get_new_feedback_event_id());
  }catch(CEventTimeoutException &e){
    /* handle the exception */
    QMessageBox msgBox;
    msgBox.setText("No connection to the node");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.exec();
  }
}

void MainWindow::on_load_sequence_clicked()
{
  QString caption("Open motion sequence file"),directory("."),filter("*.xml"),filename;
  std::string control_mode, motion_mode;
  sequence_t::step_iterator iterator;
  QFileDialog *open_file;
  QStringList file_list;
  unsigned int num=0;
  TMotionStep step;

  open_file=new QFileDialog(this,caption,directory,filter); 

  if(open_file->exec())
  {
    file_list=open_file->selectedFiles();
    if(file_list.size()==1)
    {
      try{
        filename=file_list.at(0);
        this->motion_file_name=file_list.at(0);
        this->ui.sequence_name->setText(filename.toAscii().data());
        std::cout << filename.toAscii().data() << std::endl;
        std::auto_ptr<sequence_t> seq(motion_seq(filename.toAscii().data(),xml_schema::flags::dont_validate));
        if((num=seq->num_motors())!=this->dof)
        {
          /* handle exceptions */
          throw CMotionSequenceException(_HERE_,"The number of motors in the loaded sequence does not coincide with the number of motors in the associated motor groupY");
        }
        else
        {
          // get the number of steps
          num=seq->num_steps();
          // get the control type: position or speed
          control_mode=seq->control();
          // set the control mode to all CMotorControl objects
          if(control_mode=="position")
          {
            this->ui.position_control->setChecked(true);
            this->ui.velocity_control->setChecked(false);
          }
          else
          {
            this->ui.position_control->setChecked(false);
            this->ui.velocity_control->setChecked(true);
          }
          // get the motion type: absolute or relative
          motion_mode=seq->motion();
          // set the motion mode to all CMotorControl objects
          if(motion_mode=="absolute")
          {
            this->ui.absolute_motion->setChecked(true);
            this->ui.relative_motion->setChecked(false);
          }
          else
          {
            this->ui.absolute_motion->setChecked(false);
            this->ui.relative_motion->setChecked(true);
          }
          // erase the previous sequence
          this->seq.clear();
          for(iterator=seq->step().begin();iterator!=seq->step().end();iterator++)
          {
            step.delay=iterator->delay();
            step.position.clear();
            step.velocity.clear();
            step.position=iterator->position();
            step.velocity=iterator->velocity();
            this->seq.push_back(step);
          }
          this->fill_motion_table();
        }
      }catch (const xml_schema::exception& e){
        /* handle exceptions */
        std::ostringstream os;
        os << e;
        QMessageBox msgBox;
        msgBox.setText(QString(os.str().c_str()));
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.exec();
      }
    } 
  }
  delete open_file;
}

void MainWindow::on_save_sequence_clicked()
{

}

void MainWindow::on_create_sequence_clicked()
{
  QString filename;

  if(this->ui.sequence_name.text()!="")
  {
  }
  else
  {
    QMessageBox msgBox;
    msgBox.setText("No filename specified. Impossible to create a new motion sequence file.");
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.exec();
  }
}

void MainWindow::on_execute_sequence_clicked()
{
  try{
    this->qnode.execute_sequence(this->seq);
  }catch(std::string &error){
    QMessageBox msgBox;
    msgBox.setText(error.c_str());
    msgBox.setIcon(QMessageBox::Warning);
    msgBox.exec();
  }
}

void MainWindow::on_add_step_clocked()
{

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() 
{
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace tibi_dabo_sequence_editor

