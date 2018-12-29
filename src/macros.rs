#[macro_export]
macro_rules! types {
    (
        $name:ident {
            type Vector = $v_type:ty ;
            type Matrix = $m_type:ty ;
            type Quaternion = $q_type:ty ;
        }
    ) => {
        #[derive(Debug, Clone)]
        pub enum $name {}
        unsafe impl $crate::Types for $name {
            type Vector = $v_type;
            type Matrix = $m_type;
            type Quaternion = $q_type;
        }
    };
}
