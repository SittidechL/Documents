[Firebase CRUD](https://www.youtube.com/watch?v=aUymZCxJieQ)<br>
Create data
``` 
let obj = {
        name: 'TF6',
        age: 45
        }
db.ref('users/').set(obj)
```
Read data
```
db.ref('users/user1/').on('value', (snapshot)=>{
        console.log(snapshot.val())
})
```
Update data
```
let newupdateddata={
        name:'TF0'
}
db.ref('users/user2').update(newupdateddata)
```
Deleted data
```
db.ref('users/user2').remove()
```
