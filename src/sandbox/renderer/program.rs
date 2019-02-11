use gl;
use gl::types::*;

const VERT_SOURCE: &[u8] = include_bytes!("assets/program.vert");
const FRAG_SOURCE: &[u8] = include_bytes!("assets/program.frag");

pub struct Program {
    pub(crate) program: GLuint,
    pub(crate) world: GLint,
    pub(crate) view: GLint,
    pub(crate) projection: GLint,
    pub(crate) color: GLint,
    pub(crate) lighting: GLint,
}

impl Drop for Program {
    fn drop(&mut self) {
        unsafe {
            check!(gl::DeleteProgram(self.program));
        }
    }
}

impl Program {
    pub(crate) fn new() -> Program {
        unsafe {
            let vert = create_shader(gl::VERTEX_SHADER, VERT_SOURCE);
            let frag = create_shader(gl::FRAGMENT_SHADER, FRAG_SOURCE);

            let program = check!(gl::CreateProgram());
            check!(gl::AttachShader(program, vert));
            check!(gl::AttachShader(program, frag));
            check!(gl::LinkProgram(program));
            check!(gl::DeleteShader(vert));
            check!(gl::DeleteShader(frag));

            Program {
                program,
                world: check!(gl::GetUniformLocation(
                    program,
                    "u_world\0".as_ptr() as *const GLchar
                )),
                view: check!(gl::GetUniformLocation(
                    program,
                    "u_view\0".as_ptr() as *const GLchar
                )),
                projection: check!(gl::GetUniformLocation(
                    program,
                    "u_projection\0".as_ptr() as *const GLchar
                )),
                color: check!(gl::GetUniformLocation(
                    program,
                    "u_color\0".as_ptr() as *const GLchar
                )),
                lighting: check!(gl::GetUniformLocation(
                    program,
                    "u_lighting\0".as_ptr() as *const GLchar
                )),
            }
        }
    }
}

unsafe fn create_shader(type_: GLenum, source: &[u8]) -> GLuint {
    let shader = check!(gl::CreateShader(type_));
    check!(gl::ShaderSource(
        shader,
        1,
        [source.as_ptr() as *const GLchar].as_ptr() as _,
        [source.len() as GLint].as_ptr()
    ));
    check!(gl::CompileShader(shader));

    let mut len = 0;
    let mut buffer = vec![0u8; 1024];
    check!(gl::GetShaderInfoLog(
        shader,
        buffer.capacity() as GLint,
        &mut len,
        buffer.as_mut_ptr() as *mut GLchar
    ));

    if len > 0 {
        panic!("{:?}", String::from_utf8_lossy(&buffer[..(len as usize)]));
    }

    shader
}
