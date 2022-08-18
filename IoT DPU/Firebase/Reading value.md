Reading <br>
html
```
<h1 id="reading-value"></h1>
```

js
```
db.ref('web/message/').on('value', (snapshot) => {
    var valueReading = snapshot.val();
    console.log(valueReading);
    document.getElementById("reading-value").innerHTML = valueReading;
})
```

Write <br>
html
```
<input type="text" id="email">
<button id="send">Send</button>
```

js
```
var firebaseRef = db.ref('emails');
document.querySelector('#send').addEventListener('click',()=>{
    const email = document.getElementById('email').value;
    firebaseRef.push(email);
    firebaseRef.set(email);
})
```
