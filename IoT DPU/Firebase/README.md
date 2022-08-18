[Firebase CRUD](https://www.youtube.com/watch?v=aUymZCxJieQ)<br>
Create data
``` 
let obj = {
        name: 'TF6',
        age: 45
        }
db.ref(data).set(obj)
```
Read data
```
db.ref('users/user1/').on('value', (snapshot)=>{
        console.log(snapshot.val())
})
```
