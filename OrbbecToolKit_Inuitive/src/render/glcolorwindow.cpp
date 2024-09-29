#include "glcolorwindow.h"

glColorWindow::glColorWindow(QWidget *parent) :
	QOpenGLWidget(parent)
{
    program_ = nullptr;
    rgb_texture_ = nullptr;
    depth_texture_ = nullptr;
}
glColorWindow::~glColorWindow()
{
}
void glColorWindow::initializeGL()
{
    makeCurrent();
    program_ = new QOpenGLShaderProgram(this);

    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    //glEnable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    model_matrix_.setToIdentity();

    InitShaders();
    InitTextures();
    InitData();
}
void glColorWindow::InitShaders()
{
    if (nullptr == program_)
    {
        qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << " program_ is nullptr.";
        return;
    }

    if (!program_->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/res/shader/vertex.vert"))
    {
        qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << " add vertex shader file failed.";
        return;
    }

    if (!program_->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/res/shader/fragment.frag"))
    {
        qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << " add fragment shader file failed.";
        return;
    }

    if (!program_->link())
    {
        qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << " program_ link failed.";
        return;
    }

    if (!program_->bind())
    {
        qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << " program_ bind failed.";
        return;
    }

    program_->bindAttributeLocation("aPosition", 0);
    program_->bindAttributeLocation("aCoordinate", 1);
    program_->bindAttributeLocation("a_rgb_src", 2);
    program_->bindAttributeLocation("a_depth_src", 3);
    program_->link();
    program_->bind();

    mvp_matrix_handle_ = program_->uniformLocation("uMatrix");
    vertices_handle_ = program_->attributeLocation("aPosition");
    texcoor_handle_ = program_->attributeLocation("aCoordinate");
    texcoor_rgb_handle_ = program_->attributeLocation("a_rgb_src");
    texcoor_depth_handle_ = program_->attributeLocation("a_depth_src");

    //sampler_rgb = glGetUniformLocation(program_, "sampler_rgb_");
    //sampler_depth = glGetUniformLocation(program_, "sampler_depth_");
    //hChangeType = glGetUniformLocation(program_, "vChangeType");
}

void glColorWindow::InitTextures()
{
    //texture_ = new QOpenGLTexture(QImage(":/res/image/orbbec.png").mirrored());
    rgb_texture_ = new QOpenGLTexture(QImage().mirrored());
    frame_width_ = rgb_texture_->width();
    frame_height_ = rgb_texture_->height();
    frame_w_h_ratio_ = static_cast<qreal>(frame_width_) / frame_height_;
    view_w_h_ratio_ = frame_w_h_ratio_;
    rgb_texture_->setMinificationFilter(QOpenGLTexture::Nearest);
    rgb_texture_->setMinificationFilter(QOpenGLTexture::Linear);
    rgb_texture_->setWrapMode(QOpenGLTexture::Repeat);

    depth_texture_ = new QOpenGLTexture(QImage().mirrored());
    depth_texture_->setMinificationFilter(QOpenGLTexture::Nearest);
    depth_texture_->setMinificationFilter(QOpenGLTexture::Linear);
    depth_texture_->setWrapMode(QOpenGLTexture::Repeat);
}

void glColorWindow::InitData()
{
    vec_vertices_ << QVector3D(-1 * frame_w_h_ratio_, -1, 0.0f)
                  << QVector3D(1 * frame_w_h_ratio_, -1, 0.0f)
                  << QVector3D(1 * frame_w_h_ratio_, 1, 0.0f)
                  << QVector3D(-1 * frame_w_h_ratio_, 1, 0.0f);

    vec_texcoords_ << QVector2D(0.0, 0.0)
                   << QVector2D(1.0, 0.0)
                   << QVector2D(1.0, 1.0)
                   << QVector2D(0.0, 1.0);

    view_matrix_.setToIdentity();
    view_matrix_.lookAt(QVector3D(0.0f, 0.0f, 1.01f), QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, 1.0f, 0.0f));
}
void glColorWindow::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    view_width_ = w;
    view_height_ = h;
    //float ratio = (float)w / h;
    float left = 0;
    float right = 0;
    float bottom = 0;
    float top = 0;
    float n = 1.0f;
    float f = 10.0f;
    view_w_h_ratio_ = static_cast<qreal>(w) / h;

    if (abs(view_w_h_ratio_ - frame_w_h_ratio_) < 1e-5)
    {
        // 一样大小
        top = 1.0f;
        bottom = -1.0f;
        left = -1.0 * frame_w_h_ratio_;
        right = 1.0f * frame_w_h_ratio_;
    }
    else if (view_w_h_ratio_ > frame_w_h_ratio_)
    {
        // 窗口比较宽， 两边留空
        top = 1.0f;
        bottom = -1.0f;
        left = -view_w_h_ratio_;
        right = view_w_h_ratio_;
    }
    if (view_w_h_ratio_ < frame_w_h_ratio_)
    {
        // 图片比较宽， 上下留空
        top = 1.0 * frame_w_h_ratio_ / view_w_h_ratio_;
        bottom = -1.0 * frame_w_h_ratio_ / view_w_h_ratio_;
        left = -1.0f * frame_w_h_ratio_;
        right = 1.0f * frame_w_h_ratio_;
    }

    projection_matrix_.setToIdentity();
    projection_matrix_.frustum(left, right, bottom, top, n, f);
}
bool glColorWindow::UpdateTexture(const unsigned char * rgb_data, int rgb_width, int rgb_height,const unsigned char * depth_data, int depth_width, int depth_height,bool d2c_){

    frame_height_ = rgb_height;
    frame_width_ = rgb_width;
    view_w_h_ratio_ = static_cast<qreal>(frame_width_) / frame_height_;
    if (view_w_h_ratio_ != frame_w_h_ratio_)
    {
        //qDebug() << "UpdateTexture :3.5";
        frame_w_h_ratio_ = static_cast<qreal>(frame_width_) / frame_height_;
        resizeGL(view_width_,view_height_);
    }

    vec_vertices_.clear();
    vec_vertices_ << QVector3D(-1 * frame_w_h_ratio_, -1, 0.0f)
                  << QVector3D(1 * frame_w_h_ratio_, -1, 0.0f)
                  << QVector3D(1 * frame_w_h_ratio_, 1, 0.0f)
                  << QVector3D(-1 * frame_w_h_ratio_, 1, 0.0f);
    d2c_switch=d2c_;
    QImage rgb_image_buf(rgb_data, depth_width, depth_height, QImage::Format_RGB888);

    if (nullptr != rgb_texture_)
    {
        rgb_texture_->release();
        delete  rgb_texture_;
        rgb_texture_ = nullptr;
    }
    rgb_texture_ = new QOpenGLTexture(rgb_image_buf.mirrored());
    rgb_texture_->setMinificationFilter(QOpenGLTexture::Nearest);
    rgb_texture_->setMinificationFilter(QOpenGLTexture::Linear);
    rgb_texture_->setWrapMode(QOpenGLTexture::Repeat);
    if(d2c_switch){
        QImage depth_image_buf(depth_data, depth_width, depth_height, QImage::Format_Alpha8);

        if (nullptr != depth_texture_)
        {
            depth_texture_->release();
            delete  depth_texture_;
            depth_texture_ = nullptr;
        }
        depth_texture_ = new QOpenGLTexture(depth_image_buf.mirrored());
        depth_texture_->setMinificationFilter(QOpenGLTexture::Nearest);
        depth_texture_->setMinificationFilter(QOpenGLTexture::Linear);
        depth_texture_->setWrapMode(QOpenGLTexture::Repeat);
    }
    update();
	return true;
}
void glColorWindow::paintGL()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    program_->bind();

    Draw();
    //    DrawText();

    program_->release();

    //PaintFps();
}
void glColorWindow::Draw()
{
    // vertex point
    program_->enableAttributeArray(vertices_handle_);
    program_->setAttributeArray(vertices_handle_, vec_vertices_.constData());

    // texture coords
    program_->enableAttributeArray(texcoor_handle_);
    program_->setAttributeArray(texcoor_handle_, vec_texcoords_.constData());

    // mvp matrix
    mvp_matrix_ = model_matrix_*projection_matrix_ * view_matrix_ ;
    program_->setUniformValue(mvp_matrix_handle_, mvp_matrix_);

    rgb_texture_->bind();
    program_->setUniformValue("sampler_rgb_", 0);
    depth_texture_->bind();
    program_->setUniformValue("sampler_depth_", 1);
    glDrawArrays(GL_TRIANGLE_FAN, 0, vec_vertices_.size());
    rgb_texture_->release();
    depth_texture_->release();
    program_->disableAttributeArray(vertices_handle_);
    program_->disableAttributeArray(texcoor_handle_);
}
