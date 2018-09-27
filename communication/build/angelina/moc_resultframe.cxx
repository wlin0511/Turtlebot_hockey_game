/****************************************************************************
** Meta object code from reading C++ file 'resultframe.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../angelina/resultframe.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'resultframe.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ResultFrame[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x05,
      28,   12,   12,   12, 0x05,
      35,   12,   12,   12, 0x05,
      47,   12,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
      68,   61,   12,   12, 0x0a,
      83,   12,   12,   12, 0x0a,
      99,   12,   12,   12, 0x0a,
     116,   12,   12,   12, 0x0a,
     142,   12,   12,   12, 0x0a,
     161,   12,   12,   12, 0x0a,
     178,   12,   12,   12, 0x0a,
     215,   12,   12,   12, 0x0a,
     230,   12,   12,   12, 0x08,
     241,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ResultFrame[] = {
    "ResultFrame\0\0readyToStart()\0done()\0"
    "stopTimer()\0resumeTimer()\0points\0"
    "addPoints(int)\0slotSendStart()\0"
    "slotDisqualify()\0confirmDisqualification()\0"
    "resumeOperations()\0stopOperations()\0"
    "updateElapsedTime(unsigned long int)\0"
    "sendGameOver()\0slotRead()\0slotDisconnected()\0"
};

void ResultFrame::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ResultFrame *_t = static_cast<ResultFrame *>(_o);
        switch (_id) {
        case 0: _t->readyToStart(); break;
        case 1: _t->done(); break;
        case 2: _t->stopTimer(); break;
        case 3: _t->resumeTimer(); break;
        case 4: _t->addPoints((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->slotSendStart(); break;
        case 6: _t->slotDisqualify(); break;
        case 7: _t->confirmDisqualification(); break;
        case 8: _t->resumeOperations(); break;
        case 9: _t->stopOperations(); break;
        case 10: _t->updateElapsedTime((*reinterpret_cast< unsigned long int(*)>(_a[1]))); break;
        case 11: _t->sendGameOver(); break;
        case 12: _t->slotRead(); break;
        case 13: _t->slotDisconnected(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ResultFrame::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ResultFrame::staticMetaObject = {
    { &QFrame::staticMetaObject, qt_meta_stringdata_ResultFrame,
      qt_meta_data_ResultFrame, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ResultFrame::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ResultFrame::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ResultFrame::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ResultFrame))
        return static_cast<void*>(const_cast< ResultFrame*>(this));
    return QFrame::qt_metacast(_clname);
}

int ResultFrame::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}

// SIGNAL 0
void ResultFrame::readyToStart()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void ResultFrame::done()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void ResultFrame::stopTimer()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void ResultFrame::resumeTimer()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}
QT_END_MOC_NAMESPACE
