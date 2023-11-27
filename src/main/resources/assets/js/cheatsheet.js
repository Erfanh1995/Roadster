

// i will be 0, 1
for (let i in ['a', 'b']){
    console.log(i);
}

// i will be 'a', 'b'
for (let i of ['a', 'b']){
    console.log(i);
}