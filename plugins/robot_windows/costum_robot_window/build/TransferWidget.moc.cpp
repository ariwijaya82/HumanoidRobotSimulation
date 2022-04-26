/****************************************************************************
** Meta object code from reading C++ file 'TransferWidget.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../TransferWidget.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'TransferWidget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_TransferWidget_t {
    QByteArrayData data[14];
    char stringdata0[163];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TransferWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TransferWidget_t qt_meta_stringdata_TransferWidget = {
    {
QT_MOC_LITERAL(0, 0, 14), // "TransferWidget"
QT_MOC_LITERAL(1, 15, 28), // "makeDefaultControllerWarning"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 1), // "s"
QT_MOC_LITERAL(4, 47, 15), // "restoreSettings"
QT_MOC_LITERAL(5, 63, 9), // "uninstall"
QT_MOC_LITERAL(6, 73, 14), // "sendController"
QT_MOC_LITERAL(7, 88, 13), // "remoteControl"
QT_MOC_LITERAL(8, 102, 18), // "SSHSessionComplete"
QT_MOC_LITERAL(9, 121, 14), // "SSHSessionDone"
QT_MOC_LITERAL(10, 136, 5), // "print"
QT_MOC_LITERAL(11, 142, 7), // "message"
QT_MOC_LITERAL(12, 150, 5), // "error"
QT_MOC_LITERAL(13, 156, 6) // "status"

    },
    "TransferWidget\0makeDefaultControllerWarning\0"
    "\0s\0restoreSettings\0uninstall\0"
    "sendController\0remoteControl\0"
    "SSHSessionComplete\0SSHSessionDone\0"
    "print\0message\0error\0status"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TransferWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x0a /* Public */,
       4,    0,   62,    2, 0x0a /* Public */,
       5,    0,   63,    2, 0x0a /* Public */,
       6,    0,   64,    2, 0x0a /* Public */,
       7,    0,   65,    2, 0x0a /* Public */,
       8,    0,   66,    2, 0x0a /* Public */,
       9,    0,   67,    2, 0x0a /* Public */,
      10,    2,   68,    2, 0x0a /* Public */,
      13,    1,   73,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,   11,   12,
    QMetaType::Void, QMetaType::QString,   11,

       0        // eod
};

void TransferWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TransferWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->makeDefaultControllerWarning((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->restoreSettings(); break;
        case 2: _t->uninstall(); break;
        case 3: _t->sendController(); break;
        case 4: _t->remoteControl(); break;
        case 5: _t->SSHSessionComplete(); break;
        case 6: _t->SSHSessionDone(); break;
        case 7: _t->print((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 8: _t->status((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject TransferWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_TransferWidget.data,
    qt_meta_data_TransferWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *TransferWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TransferWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_TransferWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int TransferWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
