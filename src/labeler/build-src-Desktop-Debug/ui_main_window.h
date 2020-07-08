/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QAction *action_Next;
    QAction *action_Previous;
    QAction *action_Clear;
    QAction *action_Load;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QFrame *frame_image;
    QLabel *label_image;
    QLineEdit *line_source;
    QLabel *label_source;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupbox_imageControls;
    QGridLayout *gridLayout;
    QLabel *label_value;
    QPushButton *button_previous;
    QPushButton *button_next;
    QPushButton *button_clear;
    QSpinBox *spinbox_value;
    QPushButton *button_recalculate;
    QGroupBox *groupBox;
    QCheckBox *checkBox_display_random;
    QSpinBox *spinBox_random_value;
    QSlider *horizontalSlider_overlap;
    QSpinBox *spinBox_random_count;
    QLabel *label_random;
    QLabel *label_random_count;
    QLabel *label_overlap;
    QPushButton *button_load;
    QPushButton *button_quit;
    QPushButton *button_source_browse;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QString::fromUtf8("MainWindowDesign"));
        MainWindowDesign->resize(984, 672);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindowDesign->sizePolicy().hasHeightForWidth());
        MainWindowDesign->setSizePolicy(sizePolicy);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QString::fromUtf8("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QString::fromUtf8("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QString::fromUtf8("actionAbout_Qt"));
        action_Next = new QAction(MainWindowDesign);
        action_Next->setObjectName(QString::fromUtf8("action_Next"));
        action_Next->setShortcutContext(Qt::ApplicationShortcut);
        action_Previous = new QAction(MainWindowDesign);
        action_Previous->setObjectName(QString::fromUtf8("action_Previous"));
        action_Previous->setShortcutContext(Qt::ApplicationShortcut);
        action_Clear = new QAction(MainWindowDesign);
        action_Clear->setObjectName(QString::fromUtf8("action_Clear"));
        action_Clear->setShortcutContext(Qt::ApplicationShortcut);
        action_Load = new QAction(MainWindowDesign);
        action_Load->setObjectName(QString::fromUtf8("action_Load"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
        frame_image = new QFrame(centralwidget);
        frame_image->setObjectName(QString::fromUtf8("frame_image"));
        frame_image->setFrameShape(QFrame::StyledPanel);
        frame_image->setFrameShadow(QFrame::Raised);
        label_image = new QLabel(frame_image);
        label_image->setObjectName(QString::fromUtf8("label_image"));
        label_image->setGeometry(QRect(1, 1, 640, 480));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label_image->sizePolicy().hasHeightForWidth());
        label_image->setSizePolicy(sizePolicy1);
        label_image->setPixmap(QPixmap(QString::fromUtf8("../../images/frame0000.jpg")));
        label_image->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        line_source = new QLineEdit(frame_image);
        line_source->setObjectName(QString::fromUtf8("line_source"));
        line_source->setGeometry(QRect(10, 540, 631, 27));
        label_source = new QLabel(frame_image);
        label_source->setObjectName(QString::fromUtf8("label_source"));
        label_source->setGeometry(QRect(10, 520, 141, 17));
        frame = new QFrame(frame_image);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(650, 10, 307, 491));
        QSizePolicy sizePolicy2(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy2);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        groupbox_imageControls = new QGroupBox(frame);
        groupbox_imageControls->setObjectName(QString::fromUtf8("groupbox_imageControls"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(groupbox_imageControls->sizePolicy().hasHeightForWidth());
        groupbox_imageControls->setSizePolicy(sizePolicy3);
        gridLayout = new QGridLayout(groupbox_imageControls);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_value = new QLabel(groupbox_imageControls);
        label_value->setObjectName(QString::fromUtf8("label_value"));

        gridLayout->addWidget(label_value, 5, 0, 1, 1);

        button_previous = new QPushButton(groupbox_imageControls);
        button_previous->setObjectName(QString::fromUtf8("button_previous"));

        gridLayout->addWidget(button_previous, 2, 0, 1, 1);

        button_next = new QPushButton(groupbox_imageControls);
        button_next->setObjectName(QString::fromUtf8("button_next"));
        button_next->setEnabled(true);
        QSizePolicy sizePolicy4(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(button_next->sizePolicy().hasHeightForWidth());
        button_next->setSizePolicy(sizePolicy4);

        gridLayout->addWidget(button_next, 1, 0, 1, 1);

        button_clear = new QPushButton(groupbox_imageControls);
        button_clear->setObjectName(QString::fromUtf8("button_clear"));

        gridLayout->addWidget(button_clear, 3, 0, 1, 1);

        spinbox_value = new QSpinBox(groupbox_imageControls);
        spinbox_value->setObjectName(QString::fromUtf8("spinbox_value"));
        sizePolicy.setHeightForWidth(spinbox_value->sizePolicy().hasHeightForWidth());
        spinbox_value->setSizePolicy(sizePolicy);
        spinbox_value->setMinimum(-99);
        spinbox_value->setValue(1);

        gridLayout->addWidget(spinbox_value, 6, 0, 1, 1);

        button_recalculate = new QPushButton(groupbox_imageControls);
        button_recalculate->setObjectName(QString::fromUtf8("button_recalculate"));

        gridLayout->addWidget(button_recalculate, 4, 0, 1, 1);


        verticalLayout_3->addWidget(groupbox_imageControls);

        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        checkBox_display_random = new QCheckBox(groupBox);
        checkBox_display_random->setObjectName(QString::fromUtf8("checkBox_display_random"));
        checkBox_display_random->setGeometry(QRect(20, 220, 261, 22));
        checkBox_display_random->setChecked(false);
        spinBox_random_value = new QSpinBox(groupBox);
        spinBox_random_value->setObjectName(QString::fromUtf8("spinBox_random_value"));
        spinBox_random_value->setGeometry(QRect(20, 50, 60, 27));
        spinBox_random_value->setMinimum(-99);
        spinBox_random_value->setValue(-1);
        horizontalSlider_overlap = new QSlider(groupBox);
        horizontalSlider_overlap->setObjectName(QString::fromUtf8("horizontalSlider_overlap"));
        horizontalSlider_overlap->setGeometry(QRect(20, 170, 261, 29));
        horizontalSlider_overlap->setMaximum(100);
        horizontalSlider_overlap->setSingleStep(10);
        horizontalSlider_overlap->setValue(50);
        horizontalSlider_overlap->setOrientation(Qt::Horizontal);
        horizontalSlider_overlap->setTickPosition(QSlider::TicksBelow);
        horizontalSlider_overlap->setTickInterval(10);
        spinBox_random_count = new QSpinBox(groupBox);
        spinBox_random_count->setObjectName(QString::fromUtf8("spinBox_random_count"));
        spinBox_random_count->setGeometry(QRect(20, 110, 60, 27));
        spinBox_random_count->setValue(10);
        label_random = new QLabel(groupBox);
        label_random->setObjectName(QString::fromUtf8("label_random"));
        label_random->setGeometry(QRect(20, 30, 221, 17));
        label_random_count = new QLabel(groupBox);
        label_random_count->setObjectName(QString::fromUtf8("label_random_count"));
        label_random_count->setGeometry(QRect(20, 90, 241, 17));
        label_overlap = new QLabel(groupBox);
        label_overlap->setObjectName(QString::fromUtf8("label_overlap"));
        label_overlap->setGeometry(QRect(20, 150, 261, 17));

        verticalLayout_3->addWidget(groupBox);

        button_load = new QPushButton(frame_image);
        button_load->setObjectName(QString::fromUtf8("button_load"));
        button_load->setGeometry(QRect(760, 540, 101, 27));
        button_quit = new QPushButton(frame_image);
        button_quit->setObjectName(QString::fromUtf8("button_quit"));
        button_quit->setGeometry(QRect(870, 540, 91, 27));
        button_source_browse = new QPushButton(frame_image);
        button_source_browse->setObjectName(QString::fromUtf8("button_source_browse"));
        button_source_browse->setGeometry(QRect(650, 540, 98, 27));

        hboxLayout->addWidget(frame_image);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 984, 25));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(button_next, SIGNAL(clicked()), MainWindowDesign, SLOT(button_next()));
        QObject::connect(button_previous, SIGNAL(clicked()), MainWindowDesign, SLOT(button_previous()));
        QObject::connect(button_clear, SIGNAL(clicked()), MainWindowDesign, SLOT(button_clear()));
        QObject::connect(button_load, SIGNAL(clicked()), MainWindowDesign, SLOT(button_load()));
        QObject::connect(button_quit, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));
        QObject::connect(button_source_browse, SIGNAL(clicked()), MainWindowDesign, SLOT(button_source_browse()));
        QObject::connect(checkBox_display_random, SIGNAL(stateChanged(int)), MainWindowDesign, SLOT(check_display_random()));
        QObject::connect(button_recalculate, SIGNAL(clicked()), MainWindowDesign, SLOT(button_recalculate()));

        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "ROS Labeler", 0, QApplication::UnicodeUTF8));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0, QApplication::UnicodeUTF8));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0, QApplication::UnicodeUTF8));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0, QApplication::UnicodeUTF8));
        action_Next->setText(QApplication::translate("MainWindowDesign", "&Next", 0, QApplication::UnicodeUTF8));
        action_Next->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+N", 0, QApplication::UnicodeUTF8));
        action_Previous->setText(QApplication::translate("MainWindowDesign", "&Previous", 0, QApplication::UnicodeUTF8));
        action_Previous->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+P", 0, QApplication::UnicodeUTF8));
        action_Clear->setText(QApplication::translate("MainWindowDesign", "&Clear", 0, QApplication::UnicodeUTF8));
        action_Clear->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+C", 0, QApplication::UnicodeUTF8));
        action_Load->setText(QApplication::translate("MainWindowDesign", "&Load", 0, QApplication::UnicodeUTF8));
        action_Load->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+L", 0, QApplication::UnicodeUTF8));
        label_image->setText(QString());
        line_source->setText(QString());
        label_source->setText(QApplication::translate("MainWindowDesign", "Image Source Folder", 0, QApplication::UnicodeUTF8));
        groupbox_imageControls->setTitle(QApplication::translate("MainWindowDesign", "ROI Image Controls", 0, QApplication::UnicodeUTF8));
        label_value->setText(QApplication::translate("MainWindowDesign", "ROI Selection Value", 0, QApplication::UnicodeUTF8));
        button_previous->setText(QApplication::translate("MainWindowDesign", "Previous Image", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        button_next->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_next->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_STATUSTIP
        button_next->setText(QApplication::translate("MainWindowDesign", "Next Image", 0, QApplication::UnicodeUTF8));
        button_clear->setText(QApplication::translate("MainWindowDesign", "Clear Last ROI", 0, QApplication::UnicodeUTF8));
        button_recalculate->setText(QApplication::translate("MainWindowDesign", "Recalculate ROI", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("MainWindowDesign", "Random ROI Controls", 0, QApplication::UnicodeUTF8));
        checkBox_display_random->setText(QApplication::translate("MainWindowDesign", "Display randomly generated ROI", 0, QApplication::UnicodeUTF8));
        label_random->setText(QApplication::translate("MainWindowDesign", "Random ROI Selection Value", 0, QApplication::UnicodeUTF8));
        label_random_count->setText(QApplication::translate("MainWindowDesign", "Number of random ROI to generate", 0, QApplication::UnicodeUTF8));
        label_overlap->setText(QApplication::translate("MainWindowDesign", "ROI overlap with existing ROI", 0, QApplication::UnicodeUTF8));
        button_load->setText(QApplication::translate("MainWindowDesign", "Load", 0, QApplication::UnicodeUTF8));
        button_quit->setText(QApplication::translate("MainWindowDesign", "Quit", 0, QApplication::UnicodeUTF8));
        button_source_browse->setText(QApplication::translate("MainWindowDesign", "Browse", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H
