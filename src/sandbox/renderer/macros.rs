macro_rules! check {
    ($gl:expr) => {{
        gl::GetError();
        let result = $gl;
        let error = gl::GetError();
        assert_eq!(gl::NO_ERROR, error);
        result
    }};
}

macro_rules! primitive {
    ($(
        mod $mod_name:ident {

            struct $struct_name:ident {
                path => $path:expr,
                offset0 => $offset0:expr,
                offset1 => $offset1:expr,
                indices => $ind:expr,
            }
        }
    )+) => {$(
        mod $mod_name {
            use lazy_static::*;

            use gl;
            use gl::types::*;

            const RAW_DATA: &[u8] = include_bytes!($path);

            lazy_static! {
                pub static ref INDEX: &'static [u8] = &RAW_DATA[..$offset0];
                pub static ref VERTEX: &'static [u8] = &RAW_DATA[$offset0..];
            }

            pub(crate) struct $struct_name {
                pub(crate) vbo: GLuint,
                pub(crate) ebo: GLuint,
                pub(crate) vao: GLuint,
            }

            impl Drop for $struct_name {
                fn drop(&mut self) {
                    unsafe {
                        check!(gl::DeleteBuffers(1, &self.vbo));
                        check!(gl::DeleteBuffers(1, &self.ebo));
                        check!(gl::DeleteVertexArrays(1, &self.vao));
                    }
                }
            }

            impl $struct_name {
                pub fn new() -> $struct_name {
                    unsafe {
                        let vbo = create_buffer(gl::ARRAY_BUFFER, *VERTEX);
                        let ebo = create_buffer(gl::ELEMENT_ARRAY_BUFFER, *INDEX);
                        $struct_name {
                            vbo,
                            ebo,
                            vao: create_vao(vbo, ebo),
                        }
                    }
                }

                pub fn tris(&self) -> usize {
                    self.indices() / 3
                }

                pub fn indices(&self) -> usize {
                    $ind
                }
            }

            unsafe fn create_buffer(target: GLenum, data: &[u8]) -> GLuint {
                let mut buffer = 0;
                check!(gl::CreateBuffers(1, &mut buffer));
                check!(gl::BindBuffer(target, buffer));
                check!(gl::BufferData(
                    target,
                    data.len() as _,
                    data.as_ptr() as _,
                    gl::STATIC_DRAW
                ));
                check!(gl::BindBuffer(target, 0));
                buffer
            }

            unsafe fn create_vao(vbo: GLuint, ebo: GLuint) -> GLuint {
                let mut vao = 0;
                check!(gl::CreateVertexArrays(1, &mut vao));
                check!(gl::BindVertexArray(vao));
                check!(gl::BindBuffer(gl::ARRAY_BUFFER, vbo));
                check!(gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, ebo));
                check!(gl::EnableVertexAttribArray(0));
                check!(gl::EnableVertexAttribArray(1));
                check!(gl::VertexAttribPointer(
                    0,
                    3,
                    gl::FLOAT,
                    gl::FALSE,
                    3 << 2,
                    0 as _
                ));
                let offset = std::ptr::null::<u8>().offset($offset1 - $offset0) as _;
                check!(gl::VertexAttribPointer(
                    1,
                    3,
                    gl::FLOAT,
                    gl::FALSE,
                    3 << 2,
                    offset
                ));
                check!(gl::BindVertexArray(0));
                check!(gl::DisableVertexAttribArray(0));
                check!(gl::DisableVertexAttribArray(1));
                check!(gl::BindBuffer(gl::ARRAY_BUFFER, 0));
                check!(gl::BindBuffer(gl::ELEMENT_ARRAY_BUFFER, 0));
                vao
            }
        }
    )+}
}
