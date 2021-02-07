#!/usr/bin/env python
import pickle
import itertools
import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
from sklearn.preprocessing import LabelEncoder, StandardScaler
#from sklearn import cross_validation
from sklearn import metrics
from sklearn.model_selection import train_test_split

def plot_confusion_matrix(cm, classes, normalize=False,
                          title='Confusion matrix', cmap=plt.cm.Blues):
    """This function prints and plots the confusion matrix. Normalization can
    be applied by setting `normalize=True`.
    """

    plt.imshow(cm, interpolation='nearest', cmap=cmap)
    plt.title(title)
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]

    thresh = cm.max() / 2.
    for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        plt.text(j, i, '{0:.2f}'.format(cm[i, j]),
                 horizontalalignment="center",
                 color="white" if cm[i, j] > thresh else "black")

    plt.tight_layout()
    plt.ylabel('True label')
    plt.xlabel('Predicted label')

# Load training data from disk
training_set = pickle.load(open('/home/jetson/robot/src/imagecontrol/Trainning_data/training_set_new.sav', 'rb'))

# Format the features and labels for use with scikit learn
feature_list = []
label_list = []

for item in training_set:
    if np.isnan(item[0]).sum() < 1:
        feature_list.append(item[0])
        label_list.append(item[1])

print('Features in Training Set: {}'.format(len(training_set)))
print('Invalid Features in Training set: {}'.format(len(training_set)-len(feature_list)))

X = np.array(feature_list)
# Fit a per-column scaler
X_scaler = StandardScaler().fit(X)
# Apply the scaler to X
X_train = X_scaler.transform(X)
y_train = np.array(label_list)

# Convert label strings to numerical encoding
encoder = LabelEncoder()
y_train = encoder.fit_transform(y_train)
features_train, features_test, labels_train, labels_test = train_test_split(X_train, y_train, test_size=0.1, random_state=42)

# Create classifier
clf = svm.SVC(kernel='linear', C=0.1)


clf.fit(X=features_train, y=labels_train)


# print('Accuracy score: %s' % str(accuracy_score))
y_pred = clf.predict(features_test)
print(y_pred)
accuracy_score = metrics.accuracy_score(labels_test, y_pred)
print(accuracy_score)
confusion_matrix = metrics.confusion_matrix(labels_test,y_pred)

class_names = encoder.classes_.tolist()

# #Train the classifier
# clf.fit(X=X_train, y=y_train)

model = {'classifier': clf, 'classes': encoder.classes_, 'scaler': X_scaler}

# # Save classifier to disk
pickle.dump(model, open('model.sav', 'wb'))

# # Plot non-normalized confusion matrix
plt.figure()
plot_confusion_matrix(confusion_matrix, classes=encoder.classes_,
                       title='Confusion matrix, without normalization')

# # Plot normalized confusion matrix
# plt.figure()
# plot_confusion_matrix(confusion_matrix, classes=encoder.classes_, normalize=True,
#                       title='Normalized confusion matrix')

plt.show()
