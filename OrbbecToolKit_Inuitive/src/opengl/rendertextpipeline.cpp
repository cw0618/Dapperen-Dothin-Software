﻿#include "rendertextpipeline.h"
#include "QDebug"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
/** \class CaptureDialog
*
* 文字显示类
*
*/
void RenderTextPipeLine::createProgram(){
    IPipeLine::createProgram();
    if(mProgramId != 0){
        textColorHandler  = glGetUniformLocation(mProgramId, "textColor");
        projectionHandler = glGetUniformLocation(mProgramId, "projection");
        qDebug() << "RenderTextPipeLine createProgram: " << mProgramId;

        initCharacters();
        initVAO();
    }
}

void RenderTextPipeLine::updateRenderText(QString _text, float x, float y){
    text = _text;
    pos_x = x;
    pos_y = y;
}

void RenderTextPipeLine::enableRenderText(bool enable)
{
    enableText = enable;
}

void RenderTextPipeLine::draw(){
    if(enableText){
        IPipeLine::useProgram();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glm::mat4 projection = glm::ortho(0.0f, static_cast<GLfloat>(surfaceWidth), 0.0f, static_cast<GLfloat>(surfaceHeight));
        glUniformMatrix4fv(projectionHandler, 1, GL_FALSE, glm::value_ptr(projection));

        glUniform3f(textColorHandler, r, g, b);
        glActiveTexture(GL_TEXTURE0);
        glBindVertexArray(VAO);

        float maxH = 0;
        float totalW = 0;
        for(int i = 0; i < text.length(); i++){
            char c = text.at(i).unicode();
            Character ch = characters[c];

            GLfloat w = ch.Size.x * scale;
            GLfloat h = ch.Size.y * scale;
            if(h > maxH){
                maxH = h;
            }
            totalW += (ch.Advance >> 6) * scale;
        }

        GLfloat x = pos_x;
        GLfloat y = pos_y;

        if((x + totalW) > surfaceWidth){
            x -= totalW;
        }

        if(y + maxH > surfaceHeight){
            y -= maxH;
        }

        for(int i = 0; i < text.length(); i++){
            char c = text.at(i).unicode();
            Character ch = characters[c];

            GLfloat xpos = x + ch.Bearing.x * scale;
            GLfloat ypos = y - (ch.Size.y - ch.Bearing.y) * scale;

            GLfloat w = ch.Size.x * scale;
            GLfloat h = ch.Size.y * scale;
            // Update VBO for each character
            GLfloat vertices[6][4] = {
                { xpos,     ypos + h,   0.0, 0.0 },
                { xpos,     ypos,       0.0, 1.0 },
                { xpos + w, ypos,       1.0, 1.0 },

                { xpos,     ypos + h,   0.0, 0.0 },
                { xpos + w, ypos,       1.0, 1.0 },
                { xpos + w, ypos + h,   1.0, 0.0 }
            };
            // Render glyph texture over quad
            glBindTexture(GL_TEXTURE_2D, ch.TextureID);
            // Update content of VBO memory
            glBindBuffer(GL_ARRAY_BUFFER, VBO);
            glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); // Be sure to use glBufferSubData and not glBufferData

            glBindBuffer(GL_ARRAY_BUFFER, 0);
            // Render quad
            glDrawArrays(GL_TRIANGLES, 0, 6);
            // Now advance cursors for next glyph (note that advance is number of 1/64 pixels)
            x += (ch.Advance >> 6) * scale; // Bitshift by 6 to get value in pixels (2^6 = 64 (divide amount of 1/64th pixels by 64 to get amount of pixels))
        }

        glBindVertexArray(0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_BLEND);

    }
}


void RenderTextPipeLine::initCharacters()
{
    FT_Library ft;
    if(FT_Init_FreeType(&ft)){
        qCritical() << "ERROR::FREETYPE: Could not init FreeType Library";
    }

    FT_Face face;
    if(FT_New_Face(ft, "res/fonts/arial.ttf", 0, &face)){
        qCritical() << "ERROR::FREETYPE: Failed to load font";
    }

    // Set size to load glyphs as
    FT_Set_Pixel_Sizes(face, 0, 48);

    // Disable byte-alignment restriction
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    // Load first 128 characters of ASCII set
    for(GLubyte c = 0; c < 128; c++){
        if(FT_Load_Char(face, c, FT_LOAD_RENDER)){
            qCritical("ERROR::FREETYTPE: Failed to load Glyph");
            continue;
        }

        GLuint texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RED,
            face->glyph->bitmap.width,
            face->glyph->bitmap.rows,
            0,
            GL_RED,
            GL_UNSIGNED_BYTE,
            face->glyph->bitmap.buffer
        );

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // Now store character for later use
        Character character = {
            texture,
            glm::ivec2(face->glyph->bitmap.width, face->glyph->bitmap.rows),
            glm::ivec2(face->glyph->bitmap_left, face->glyph->bitmap_top),
            face->glyph->advance.x
        };
        characters.insert(std::pair<GLchar, Character>(c, character));
    }

    glBindTexture(GL_TEXTURE_2D, 0);
    // Destroy FreeType once we're finished
    FT_Done_Face(face);
    FT_Done_FreeType(ft);
}

void RenderTextPipeLine::initVAO()
{
    // Configure VAO/VBO for texture quads
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
