use cgmath::{prelude::*, Matrix4, PerspectiveFov, Point3, Vector3};
use stb::image::Image;

use gl;
use gl::types::*;

use std::{ffi::CString, ptr};

mod cube {
    static BLOB: &[u8; 912] = include_bytes!("assets/cube.bin");

    lazy_static! {
        pub static ref INDEX_DATA: &'static [u8] = &BLOB[..144];
        pub static ref VERTEX_DATA: &'static [u8] = &BLOB[144..];
    }
}

pub type RenderResult<T> = ::std::result::Result<T, RenderError>;

pub type Color3 = ::cgmath::Vector3<f32>;

#[derive(Debug)]
pub enum RenderError {
    VertexShaderCompile(String),
    FragmentShaderCompile(String),
    Error(GLenum),
}

#[derive(Debug)]
pub struct ShaderProgram {
    program_id: GLuint,
    projection_uniform: GLint,
    view_uniform: GLint,
    world_uniform: GLint,
    color_uniform: GLint,
    light_uniform: GLint,
    texture_uniform: GLint,
}

#[derive(Debug)]
pub struct Renderer {
    program: ShaderProgram,

    // box mesh
    box_vbo: GLuint,
    box_ebo: GLuint,

    // textures
    diamond_texture: GLuint,
}

pub struct Camera {
    perspective: PerspectiveFov<f32>,

    eye: Point3<f32>,
    center: Point3<f32>,
    up: Vector3<f32>,
}

impl Camera {
    pub fn new(perspective: PerspectiveFov<f32>) -> Self {
        Self {
            perspective,
            eye: Point3::new(0.0, 0.0, 0.0),
            center: Point3::new(0.0, 0.0, 0.0),
            up: Vector3::new(0.0, 1.0, 0.0),
        }
    }

    pub fn update_view(&mut self, eye: Point3<f32>, center: Point3<f32>, up: Vector3<f32>) {
        self.eye = eye;
        self.center = center;
        self.up = up;
    }

    pub fn projection(&self) -> Matrix4<f32> {
        self.perspective.into()
    }

    pub fn view(&self) -> Matrix4<f32> {
        Matrix4::look_at(self.eye, self.center, self.up)
    }
}

impl Renderer {
    pub fn new() -> RenderResult<Self> {
        // set up mesh and attribute pointers
        unsafe {
            let (box_vbo, box_ebo) = Self::create_box_buffers()?;

            Ok(Self {
                box_vbo: box_vbo,
                box_ebo: box_ebo,

                program: Self::create_shader_program()?,
                diamond_texture: Self::create_diamond_texture()?,
            })
        }
    }

    unsafe fn init_state(&self) {
        // set shader program
        gl::UseProgram(self.program.program_id);

        gl::BindBuffer(gl::ARRAY_BUFFER, self.box_vbo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, self.box_ebo);

        // set up attributes
        gl::EnableVertexAttribArray(0);
        gl::EnableVertexAttribArray(1);
        gl::EnableVertexAttribArray(2);

        gl::VertexAttribPointer(0, 3, gl::FLOAT, gl::FALSE, 3 << 2, 0 as _);
        gl::VertexAttribPointer(
            1,
            3,
            gl::FLOAT,
            gl::FALSE,
            3 << 2,
            ptr::null::<u8>().offset(432 - 144) as _,
        );
        gl::VertexAttribPointer(
            2,
            2,
            gl::FLOAT,
            gl::FALSE,
            2 << 2,
            ptr::null::<u8>().offset(720 - 144) as _,
        );
    }

    unsafe fn create_shader_program() -> RenderResult<ShaderProgram> {
        gl::GetError();

        let mut info_log = vec![0u8; 1024];
        let mut info_log_len = 0;

        let vertex_shader = gl::CreateShader(gl::VERTEX_SHADER);
        let vertex_shader_source = include_str!("assets/shader.vert");

        gl::ShaderSource(
            vertex_shader,
            1,
            [vertex_shader_source.as_ptr()].as_ptr() as *const *const GLchar,
            [vertex_shader_source.len()].as_ptr() as _,
        );
        gl::CompileShader(vertex_shader);
        gl::GetShaderInfoLog(
            vertex_shader,
            1024,
            &mut info_log_len,
            info_log.as_mut_ptr() as _,
        );

        if info_log_len > 0 {
            return Err(RenderError::VertexShaderCompile(
                String::from_utf8_lossy(&info_log[..info_log_len as usize]).into_owned(),
            ));
        }

        let fragment_shader = gl::CreateShader(gl::FRAGMENT_SHADER);
        let fragment_shader_source = include_str!("assets/shader.frag");

        gl::ShaderSource(
            fragment_shader,
            1,
            [fragment_shader_source.as_ptr()].as_ptr() as *const *const GLchar,
            [fragment_shader_source.len()].as_ptr() as _,
        );
        gl::CompileShader(fragment_shader);
        gl::GetShaderInfoLog(
            fragment_shader,
            1024,
            &mut info_log_len,
            info_log.as_mut_ptr() as _,
        );

        if info_log_len > 0 {
            return Err(RenderError::FragmentShaderCompile(
                String::from_utf8_lossy(&info_log[..info_log_len as usize]).into_owned(),
            ));
        }

        let program_id = gl::CreateProgram();
        gl::AttachShader(program_id, vertex_shader);
        gl::AttachShader(program_id, fragment_shader);
        gl::LinkProgram(program_id);

        gl::DeleteShader(vertex_shader);
        gl::DeleteShader(fragment_shader);
        gl::UseProgram(program_id);

        let view_uniform = gl::GetUniformLocation(program_id, "u_view\0".as_ptr() as _);
        let projection_uniform = gl::GetUniformLocation(program_id, "u_projection\0".as_ptr() as _);
        let world_uniform = gl::GetUniformLocation(program_id, "u_world\0".as_ptr() as _);
        let color_uniform = gl::GetUniformLocation(program_id, "u_color\0".as_ptr() as _);
        let light_uniform = gl::GetUniformLocation(program_id, "u_light\0".as_ptr() as _);
        let texture_uniform = gl::GetUniformLocation(program_id, "u_texture\0".as_ptr() as _);

        let error = gl::GetError();
        if error != gl::NO_ERROR {
            Err(RenderError::Error(error))
        } else {
            Ok(ShaderProgram {
                program_id,
                projection_uniform,
                view_uniform,
                world_uniform,
                color_uniform,
                light_uniform,
                texture_uniform,
            })
        }
    }

    unsafe fn create_diamond_texture() -> RenderResult<GLuint> {
        let mut texture = 0;
        gl::GenTextures(1, &mut texture);

        gl::ActiveTexture(gl::TEXTURE0);
        gl::BindTexture(gl::TEXTURE_2D, texture);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_S, gl::REPEAT as _);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_WRAP_T, gl::REPEAT as _);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MAG_FILTER, gl::NEAREST as _);
        gl::TexParameteri(gl::TEXTURE_2D, gl::TEXTURE_MIN_FILTER, gl::NEAREST as _);

        // decode image
        let image: Image<u8> = Image::from_file("examples/assets/diamonds.png", 3).unwrap();
        gl::TexImage2D(
            gl::TEXTURE_2D,
            0,
            gl::RGB8 as _,
            image.width() as _,
            image.height() as _,
            0,
            gl::RGB,
            gl::UNSIGNED_BYTE,
            image.as_ptr() as _,
        );

        let error = gl::GetError();
        if error != gl::NO_ERROR {
            Err(RenderError::Error(error))
        } else {
            Ok(texture)
        }
    }

    unsafe fn create_box_buffers() -> RenderResult<(GLuint, GLuint)> {
        gl::GetError();

        let mut box_vbo = 0;
        let mut box_ebo = 0;

        gl::GenBuffers(1, &mut box_vbo);
        gl::GenBuffers(1, &mut box_ebo);

        gl::BindBuffer(gl::ARRAY_BUFFER, box_vbo);
        gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, box_ebo);

        gl::BufferData(
            gl::ARRAY_BUFFER,
            cube::VERTEX_DATA.len() as _,
            cube::VERTEX_DATA.as_ptr() as _,
            gl::STATIC_DRAW,
        );
        gl::BufferData(
            gl::ELEMENT_ARRAY_BUFFER,
            cube::INDEX_DATA.len() as _,
            cube::INDEX_DATA.as_ptr() as _,
            gl::STATIC_DRAW,
        );

        let error = gl::GetError();
        if error != gl::NO_ERROR {
            Err(RenderError::Error(error))
        } else {
            Ok((box_vbo, box_ebo))
        }
    }

    pub fn clear(&self) -> RenderResult<()> {
        unsafe {
            self.init_state();

            gl::ClearColor(0.24, 0.24, 0.24, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

            gl::Enable(gl::DEPTH_TEST);

            gl::ActiveTexture(gl::TEXTURE0);
            gl::BindTexture(gl::TEXTURE_2D, self.diamond_texture);
            gl::Uniform1i(self.program.texture_uniform, 0);

            check_gl_error()
        }
    }

    pub fn set_light(&self, dir: Vector3<f32>) -> RenderResult<()> {
        unsafe {
            gl::Uniform3f(self.program.light_uniform, dir.x, dir.y, dir.z);

            check_gl_error()
        }
    }

    pub fn set_camera(&self, camera: &Camera) -> RenderResult<()> {
        unsafe {
            gl::UniformMatrix4fv(
                self.program.projection_uniform,
                1,
                gl::FALSE,
                camera.projection().as_ptr(),
            );
            gl::UniformMatrix4fv(
                self.program.view_uniform,
                1,
                gl::FALSE,
                camera.view().as_ptr(),
            );
            check_gl_error()
        }
    }

    pub fn render_box(&self, transform: Matrix4<f32>, color: Vector3<f32>) -> RenderResult<()> {
        unsafe {
            gl::UniformMatrix4fv(self.program.world_uniform, 1, gl::FALSE, transform.as_ptr());
            gl::Uniform3f(self.program.color_uniform, color.x, color.y, color.z);

            gl::DrawElements(gl::TRIANGLES, 36, gl::UNSIGNED_INT, ptr::null());

            check_gl_error()
        }
    }
}

#[inline]
unsafe fn check_gl_error() -> RenderResult<()> {
    let error = gl::GetError();
    if error != gl::NO_ERROR {
        Err(RenderError::Error(error))
    } else {
        Ok(())
    }
}

impl Drop for Renderer {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteTextures(1, &mut self.diamond_texture);
            gl::DeleteBuffers(1, &self.box_vbo);
            gl::DeleteBuffers(1, &self.box_ebo);
        }
    }
}

impl Drop for ShaderProgram {
    fn drop(&mut self) {
        unsafe {
            gl::DeleteProgram(self.program_id);
        }
    }
}
