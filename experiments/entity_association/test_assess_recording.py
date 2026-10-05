"""Protect the evaluation against counting waiting rows as one identity."""
import unittest

from experiments.entity_association.assess_recording import compare, score


def lab(i, name):
    return {'observation_id': str(i), 'physical_object': name}


class AccountingTests(unittest.TestCase):
    def test_waiting_does_not_mean_together(self):
        rows = [lab(1, 'painting'), lab(2, 'painting'), lab(3, 'chair')]
        result = score(rows, {1: None, 2: None, 3: None})
        self.assertEqual(result['same_together'], 0)
        self.assertEqual(result['same_apart'], 1)
        self.assertEqual(result['different_together'], 0)
        self.assertEqual(result['different_apart'], 2)
        self.assertEqual(result['waiting'], 3)

    def test_groups_report_new_errors_and_keep_old_errors(self):
        rows = [lab(1, 'painting'), lab(2, 'painting'), lab(3, 'chair'), lab(4, 'lamp')]
        # Painting/chair is already wrong. Grouping recovers the second painting,
        # but also connects it to the chair. A waiting lamp stays waiting.
        result = compare(rows, {1: 'a', 2: 'b', 3: 'a', 4: None}, {'a': 'a', 'b': 'a'})
        self.assertEqual(result['baseline']['wrong_pairs'], [[1, 3]])
        self.assertEqual(result['added_same_pairs'], 1)
        self.assertEqual(result['added_wrong_pairs'], [[2, 3]])
        self.assertEqual(result['grouped']['different_together'], 2)
        self.assertEqual(result['grouped']['waiting'], 1)

    def test_object_counts_include_singletons_and_all_waiting(self):
        result = score([lab(1, 'painting'), lab(2, 'painting'), lab(3, 'chair')],
                       {1: 'a', 2: 'b', 3: None})
        self.assertEqual(result['objects'], 2)
        self.assertEqual(result['per_object']['painting']['pieces'], 2)
        self.assertEqual(result['per_object']['chair']['pieces'], 0)
        self.assertEqual(result['per_object']['chair']['waiting'], 1)


if __name__ == '__main__':
    unittest.main()
