Reading <br>
html
```html
<h1 id="reading-value"></h1>
```

js
```js
db.ref('web/message/').on('value', (snapshot) => {
    var valueReading = snapshot.val();
    console.log(valueReading);
    document.getElementById("reading-value").innerHTML = valueReading;
})
```

Write <br>
https://www.youtube.com/watch?v=VXWmJsv1Vh4&t=406s <br>
html
```html
<input type="text" id="email">
<button id="send">Send</button>
```

js
```js
var firebaseRef = db.ref('emails');
document.querySelector('#send').addEventListener('click',()=>{
    const email = document.getElementById('email').value;
    firebaseRef.push(email);
    firebaseRef.set(email);
})
```