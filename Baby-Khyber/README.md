## An attempt at simplifying Khyber (PKC) from IBM ##
## Takeaways 
1. Khyber utilises lattice polynomials to find a balance between storage and security. 

2. Math Problems like the Shortest Vector Problem and Learning With Errors Problem, along with high dimensionality makes it difficult to break Khyber. (Check out Ajtai's worst case to average case reduction)

3. Additional concepts like circulant matrices, Parallelopipeds and progress from vectors, polynomials to polynomial rings.

4. Transition and adoption of Khyber as a Key Encapsulation Mechanism i.e. Okamoto Transform, still slow due to current pervasive legacy systems.

5. However, it is effective against Quantum Computers that renders classical algorithms vulnerable. (Check out Grover's Algorithm)


## Notes
- Commented out the attacks.py due to difficulties mimicking BKZ attack 
- Apologies for the use of AI!

## References
Professor Alfred Menezes, University of Waterloo, was of great help in my pursuit of understanding the Mathematics of PQC as well as the algorithms.

- The math: https://youtube.com/playlist?list=PLA1qgQLL41STNFDvPJRqrHtuz0PIEJ4a8&si=twLl-rs2Qfadvzx1
- The Algorithms: https://youtube.com/playlist?list=PLA1qgQLL41SSUOHlq8ADraKKzv47v2yrF&si=30SYacDy5gsxU7q_
