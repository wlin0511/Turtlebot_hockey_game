/****************************************************************************
** Meta object code from reading C++ file 'editplaygrounddialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../angelina/editplaygrounddialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'editplaygrounddialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_EditPlayGroundDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      22,   21,   21,   21, 0x0a,
      37,   31,   21,   21, 0x08,
      58,   31,   21,   21, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_EditPlayGroundDialog[] = {
    "EditPlayGroundDialog\0\0accept()\0value\0"
    "slotAChanged(double)\0slotBChanged(double)\0"
};

void EditPlayGroundDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        EditPlayGroundDialog *_t = static_cast<EditPlayGroundDialog *>(_o);
        switch (_id) {
        case 0: _t->accept(); break;
        case 1: _t->slotAChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->slotBChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData EditPlayGroundDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject EditPlayGroundDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_EditPlayGroundDialog,
      qt_meta_data_EditPlayGroundDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &EditPlayGroundDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *EditPlayGroundDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *EditPlayGroundDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_EditPlayGroundDialog))
        return static_cast<void*>(const_cast< EditPlayGroundDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int EditPlayGroundDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
