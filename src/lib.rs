#[cfg(test)]
mod tests {
    #[test]
    fn basic_test() {
        assert!(true);
    }

    #[test]
    fn string_test() {
        let greeting = "Hello, World!";
        assert_eq!(greeting, "Hello, World!");
    }

    #[test]
    fn arithmetic_test() {
        assert_eq!(2 + 2, 4);
        assert!(5 > 3);
    }
}
