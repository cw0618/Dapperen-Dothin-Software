#include "mplaintextedit.h"
#include <QDebug>
#include <QTextBlock>
MPlainTextEdit::MPlainTextEdit(QWidget *parent) : QPlainTextEdit(parent)
{
    installEventFilter(this);
}

void MPlainTextEdit::showEvent(QShowEvent *event)
{
    Q_UNUSED(event);

    if(waitForCommand){
        setFocus();
    }else{
        clearFocus();
    }
}

bool MPlainTextEdit::eventFilter(QObject *watched, QEvent *event)
{
    if(watched == this) {
        if(event->type() == QEvent::KeyPress){

            if(!waitForCommand){
                return true;
            }

            QKeyEvent *k = static_cast<QKeyEvent *>(event);

            switch(k->key()){
            case Qt::Key_Return:
            case Qt::Key_Enter:
                mNoticeCmd();
                return true;

            case Qt::Key_Up:
                return true;
            case Qt::Key_Down:
                return true;

            case Qt::Key_Left:
            case Qt::Key_Backspace:
                if(textCursor().positionInBlock() <= currentTag.length()){
                    return true;
                }
                break;

            case Qt::Key_Right:
            case Qt::Key_Delete:
                break;

            case Qt::Key_Home:
                {
                    QTextCursor cursor = textCursor();
                    cursor.movePosition(QTextCursor::StartOfLine);
                    cursor.movePosition(QTextCursor::Right, QTextCursor::MoveAnchor, currentTag.length());
                    setTextCursor(cursor);
                }
                return true;
            case Qt::Key_End:
                break;

            case Qt::Key_PageDown:
            case Qt::Key_PageUp:
                return true;

            default:
//                if(textCursor().positionInBlock() <= currentTag.length()){
//                    return true;
//                }

                break;
            }
        }
    }

    return QWidget::eventFilter(watched,event);
}

void MPlainTextEdit::mNoticeCmd()
{
    QString lastBlock = document()->lastBlock().text();
    QString cmd = lastBlock.right(lastBlock.length() - currentTag.length());
    clearFocus();
    emit cmdStr(cmd);
}

void MPlainTextEdit::mOnShowInfo(QString info)
{
    appendPlainText(info);
}

void MPlainTextEdit::mOnWaitCommand(QString tag)
{
    waitForCommand = true;
    currentTag = tag;

    mOnShowInfo(tag);
    QTextCursor cursor = textCursor();
    cursor.movePosition(QTextCursor::End);
    setTextCursor(cursor);

    if(this->isVisible()){
        setFocus();
    }
}


void MPlainTextEdit::inputMethodEvent(QInputMethodEvent *event)
{
    if(waitForCommand){
        //杈撳叆娉曡緭鍏ョ殑鏁版嵁
        QPlainTextEdit::inputMethodEvent(event);
    }
}

void MPlainTextEdit::mousePressEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
}

void MPlainTextEdit::mouseDoubleClickEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
}
