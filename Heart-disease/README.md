# Heart-disease
Cardiovascular disease prediction SC1015 Mini-Project
## About
This is a mini-project for SC1015(INTRO TO DATA SCI & ART INTELL) which focuses on cardiovascular diseases in the United States of America. The dataset to be examined is from [2022 annual CDC survey data.](https://www.kaggle.com/datasets/kamilpytlak/personal-key-indicators-of-heart-disease?rvi=1)
For a detailed walkthrough, please view the source code in order from:
1. Data Exploration and Extraction
2. Data Visualisation
3. Machine Learning Models
## Contributors
- Chester Chan
- Jothilingam Dheeraj
- Liu Han Wen
## Problem Definition
- How can we identify the communities in America with a high-risk for cardiovascular diseases?
- Which models are best suited for this prediction?
## Models Used
1. Logistic Regression
2. Gradient Boosting
3. Random Forest
4. Decision tree
## Conclusion
- America has a median and mean BMI higher than the recommended healthy BMI range, suggesting obesity prevalent in all states. Additionally, even after the new, height-adjusted BMI was used, BMI still remained high.
- Age positively correlates to the risk of cardiovascular diseases, White people had the highest percentage of cardiovasular diseases with Hispanic people having the lowest, Arkansas was the state with the highest rates while Virgin Islands had the lowest.
- Given the medical nature of the problem, the false negative rates would be of highest concern since medical professionals want to reduce the number of undetected positive cases. Surprisingly, Decision tree yields the lowest False Negative rate at 0.75 despite having the lowest accuracy rate, thus using the decison tree model might still be feasible in disease prediction.
- Future possibilities may include measuring the individual analysis for each state to focus on the most predictive factor for each state. Given how narrow the scope is, more factors can be included besides the ones used in this dataset.
  
## Takeaways
- ML Models yield different accuracy rates however for this analysis, Logistic regression, gradient boosting and random forest had the same rates and were more accurate than decision tree.
- Correlation matrices show the various relationships between the numerical variables, while performing individual barplots for each categorical variables provide easy visualisation.
- ML Models like SVM and KNN were difficult to implement given the huge number of data entries.
 
## References
- https://www.kaggle.com/datasets/kamilpytlak/personal-key-indicators-of-heart-disease?rvi=1
- https://people.maths.ox.ac.uk/trefethen/bmi.html
- https://www.bbc.com/news/uk-england-leeds-44488822
- https://towardsdatascience.com/a-look-at-precision-recall-and-f1-score-36b5fd0dd3ec
- https://professional.heart.org/en/science-news/heart-disease-and-stroke-statistics-2023-update
