package com.itheima;

import com.itheima.Springboot09SsmApplication;
import org.junit.jupiter.api.Test;
import org.junit.runner.RunWith;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.data.redis.RedisProperties;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.data.redis.connection.DataType;
import org.springframework.data.redis.connection.RedisConnection;
import org.springframework.data.redis.connection.RedisConnectionFactory;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.data.redis.core.ValueOperations;
import org.springframework.test.context.junit4.SpringJUnit4ClassRunner;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import redis.clients.jedis.Jedis;

import javax.annotation.Resource;
import java.util.List;
import java.util.Set;
@SpringBootTest(classes = Springboot09SsmApplication.class)
//@runWith注解作用：
//--@RunWith就是一个运行器
//--@RunWith(JUnit4.class)就是指用JUnit4来运行
//--@RunWith(SpringJUnit4ClassRunner.class)，让测试运行于Spring测试环 境，以便在测试开始的时候自动创建Spring的应用上下文
//--@RunWith(Suite.class)的话就是一套测试集合
//@RunWith(SpringJUnit4ClassRunner.class)
public class RedisTest {
    //操作redis中key相关
    @Autowired
    private StringRedisTemplate stringRedisTemplate;

    @Test
    public void testRedis1() {
//        stringRedisTemplate.delete("name");//删除一个key
//        Boolean hasKey = stringRedisTemplate.hasKey("name");//删除一个key
//        DataType name = stringRedisTemplate.type("name");//判断key所对应值的类型
        Set<String> keys = stringRedisTemplate.keys("*");//获取redis中所有值
//        Boolean haikey = stringRedisTemplate.delete("age1");
//        System.out.println(haikey);
        assert keys != null;
        for (String key : keys) {
            System.out.println(key);        }
        keys.forEach(key -> System.out.println("key = " + key));
        stringRedisTemplate.opsForValue().set("age", "小陈");
//        stringRedisTemplate.opsForGeo().add()
        List<String> names = stringRedisTemplate.opsForList().range("names", 0, -1);
        //Long expire = stringRedisTemplate.getExpire("age");//获取key超时时间 -1 永不超时 -2 key不存在 》=0 过期时间
        // stringRedisTemplate.randomKey();//在redis中随机获取一个key
        // stringRedisTemplate.rename("age", "age11");//给key改名· q+
        //stringRedisTemplate.move("age", 1);//移动key到指定库
    }


}



