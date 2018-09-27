/****************************************************************************
** Meta object code from reading C++ file 'testgui.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../example/testgui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'testgui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_TestGui[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      23,    8,    8,    8, 0x08,
      41,    8,    8,    8, 0x08,
      58,    8,    8,    8, 0x08,
      75,    8,    8,    8, 0x08,
      91,    8,    8,    8, 0x08,
     118,    8,    8,    8, 0x08,
     136,    8,    8,    8, 0x08,
     153,    8,    8,    8, 0x08,
     169,    8,    8,    8, 0x08,
     190,    8,    8,    8, 0x08,
     209,  205,    8,    8, 0x08,
     237,    8,    8,    8, 0x08,
     256,    8,    8,    8, 0x08,
     276,    8,    8,    8, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TestGui[] = {
    "TestGui\0\0slotConnect()\0slotReportReady()\0"
    "slotReportDone()\0slotReportGoal()\0"
    "slotSendAlive()\0slotToggleAliveTimer(bool)\0"
    "slotTellAbRatio()\0slotTellEgoPos()\0"
    "slotGameStart()\0slotDetectionStart()\0"
    "slotGameOver()\0a,b\0slotAbValues(double,double)\0"
    "slotStopMovement()\0slotTellTeamColor()\0"
    "slotTeamColor(TeamColor)\0"
};

void TestGui::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TestGui *_t = static_cast<TestGui *>(_o);
        switch (_id) {
        case 0: _t->slotConnect(); break;
        case 1: _t->slotReportReady(); break;
        case 2: _t->slotReportDone(); break;
        case 3: _t->slotReportGoal(); break;
        case 4: _t->slotSendAlive(); break;
        case 5: _t->slotToggleAliveTimer((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->slotTellAbRatio(); break;
        case 7: _t->slotTellEgoPos(); break;
        case 8: _t->slotGameStart(); break;
        case 9: _t->slotDetectionStart(); break;
        case 10: _t->slotGameOver(); break;
        case 11: _t->slotAbValues((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 12: _t->slotStopMovement(); break;
        case 13: _t->slotTellTeamColor(); break;
        case 14: _t->slotTeamColor((*reinterpret_cast< TeamColor(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData TestGui::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject TestGui::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_TestGui,
      qt_meta_data_TestGui, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TestGui::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TestGui::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TestGui::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TestGui))
        return static_cast<void*>(const_cast< TestGui*>(this));
    return QWidget::qt_metacast(_clname);
}

int TestGui::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
